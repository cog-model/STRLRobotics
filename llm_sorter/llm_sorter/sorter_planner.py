from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import List, Optional, Union
from contextlib import asynccontextmanager

import hydra
from hydra.core.config_store import ConfigStore
from hydra.utils import instantiate
import inflect
from pydantic import BaseModel

from llm_sorter.models import APIModel, BaseLLMModel
from llm_sorter import WandbLogger
from llm_sorter.environment import ItemsEvent
from llm_sorter.datasets import (
    LLPTask,
    HLPTask,
    LLPStep,
    BaseTask,
    VCTask,
    SpecTask,
    CleanRoomTask,
)
from llm_sorter.infrastructure import SorterPlannerConfig
from llm_sorter.gen_methods import BasePlanGeneration, FullPlanGeneration
from llm_sorter.processors import (
    LLPProcessor,
    HLPProcessor,
    ValidCheckProcessor,
    SpecProcessor,
    CleanRoomProcessor,
)
from asyncio import Queue
import asyncio
from fastapi import FastAPI
import uvicorn


cs = ConfigStore.instance()
cs.store(name="sorter_planner", node=SorterPlannerConfig)


class PlannerOutputType(Enum):
    RobotAction = "RobotAction"
    FeedbackRequest = "FeedbackRequest"
    UnvalidTask = "UnvalidTask"
    HumanMsg = "HumanMsg"
    Done = "Done"
    CleanAllRoom = "CleanAllRoom"


class HumanMsgType(Enum):
    UnvalidTask = "UnvalidTask"
    PlannerIsBusy = "PlannerIsBusy"
    FeedbackIsNotNeeded = "FeedbackIsNotNeeded"


@dataclass
class PlannerOutput:
    data: [str, LLPStep]
    type: PlannerOutputType

    def __str__(self):
        return f"[{self.type.value}] {self.data}"


@dataclass
class HumanMsg:
    data: str
    type: HumanMsgType

    def __str__(self) -> str:
        return f"[{self.type.value}] {self.data}"


class SorterPlanner:
    def __init__(
        self,
        logger: WandbLogger,
        llp_processor: LLPProcessor,
        hlp_processor: HLPProcessor,
        vc_processor: ValidCheckProcessor,
        spec_processor: SpecProcessor,
        clean_room_processor: CleanRoomProcessor,
        model: BaseLLMModel,
        gen_method: FullPlanGeneration,
        **kwargs,
    ) -> None:
        self._logger: WandbLogger = logger
        self._model: BaseLLMModel = model
        self._p = inflect.engine()

        self._llp_processor: LLPProcessor = llp_processor
        self._hlp_processor: HLPProcessor = hlp_processor
        self._vc_processor: ValidCheckProcessor = vc_processor
        self._spec_processor: SpecProcessor = spec_processor
        self._clean_room_processor: CleanRoomProcessor = clean_room_processor

        self._gen_method: BasePlanGeneration = gen_method

        self.reset()

    def reset(self):
        self._robot_q: Queue[LLPStep] = Queue()
        self._human_q: Queue[HumanMsg] = Queue()
        self._llm_q: Queue[BaseTask] = Queue()
        self._waiting_q: Queue[HLPTask] = Queue()

        self._reset: bool = False
        self._logger.info("[Reset]")

    async def add_task(self, goal: str) -> None:
        if not self._llm_q.empty() or not self._robot_q.empty():
            msg = HumanMsg(data="Planner is busy", type=HumanMsgType.PlannerIsBusy)
            self._human_q.put_nowait(msg)
        else:
            await self._llm_q.put(CleanRoomTask(goal=goal))
            self._logger.info(f"[Added HLP task] {goal}")

    async def do_planning(self):
        while True:
            # while not self._robot_q.empty() and not self._reset:
            # output = self._robot_q.get_nowait()
            # await self.process_output(output)

            if not self._llm_q.empty() and not self._reset:
                task = await self._llm_q.get()
                await self.process_task(task)

            # while not self._human_q.empty() and not self._reset:
            #     msg = await self._human_q.get()
            #     await self.process_human_msg(msg)

            if self._reset:
                self.reset()

            await asyncio.sleep(0.1)

    async def get_planner_output(self) -> Optional[PlannerOutput]:
        if not self._human_q.empty():
            output = await self._human_q.get()
            self._logger.info(f"[Human msg] {output}")
            output = PlannerOutput(data=output.data, type=PlannerOutputType.HumanMsg)
            return output
        elif not self._robot_q.empty():
            output = await self._robot_q.get()
            self._logger.info(f"[Processed output] {output}")
            return output
        elif self._llm_q.empty() and self._robot_q.empty() and self._human_q.empty():
            self._logger.info("[Error] Planner is empty")
            output = PlannerOutput(data="empty", type=PlannerOutputType.HumanMsg)
            return output
        else:
            return None

    async def valid_check(self, goal: str) -> None:
        task = VCTask(goal=goal)
        pred_vc_task: VCTask = self._gen_method.predict(task, self._vc_processor)
        return pred_vc_task.valid_plan

    async def add_llp_task(self, goal: str) -> None:
        if not self._llm_q.empty() or not self._robot_q.empty():
            msg = HumanMsg(data="Planner is busy", type=HumanMsgType.PlannerIsBusy)
            await self._human_q.put(msg)
        else:
            await self._llm_q.put(LLPTask(goal=goal))
            self._logger.info(f"[Added LLP task] {goal}")

    async def add_feedback(self, feedback: List[str]) -> None:
        if self._waiting_q.empty():
            msg = HumanMsg(
                data="Feedback is not needed", type=HumanMsgType.FeedbackIsNotNeeded
            )
            await self._human_q.put(msg)
        else:
            hlp_task: HLPTask = await self._waiting_q.get()
            # add feedback
            hlp_task.feedback = ItemsEvent(items=feedback)
            await self._llm_q.put(hlp_task)

    async def process_task(self, task: Union[VCTask, HLPTask, LLPTask]):
        """Process a given task based on its type with the LLM model.

        Args:
            task (Union[VCTask, HLPTask, LLPTask]): The task to be processed.
        Raises:
            ValueError: If the task type is not recognized.
        """
        if isinstance(task, CleanRoomTask):
            pred_cr_task: CleanRoomTask = self._gen_method.predict(
                task, self._clean_room_processor
            )
            self._logger.info(f"[CR] {task.goal} -> {pred_cr_task.clean_room}")

            if pred_cr_task.clean_room:
                await self._robot_q.put(
                    PlannerOutput(
                        data="CleanAllRoom", type=PlannerOutputType.CleanAllRoom
                    )
                )
            else:
                await self._llm_q.put(VCTask(goal=task.goal))
        elif isinstance(task, VCTask):
            pred_vc_task: VCTask = self._gen_method.predict(task, self._vc_processor)
            self._logger.info(f"[VC] {task.goal} -> {pred_vc_task.valid_plan}")

            if pred_vc_task.valid_plan:
                await self._llm_q.put(SpecTask(goal=task.goal))
            else:
                await self._robot_q.put(
                    PlannerOutput(data="Unvalid task", type=PlannerOutputType.UnvalidTask)
                )
        elif isinstance(task, SpecTask):
            pred_spec_task: SpecTask = self._gen_method.predict(
                task, self._spec_processor
            )
            self._logger.info(f"[Spec] {task.goal} -> {pred_spec_task.request}")

            if pred_spec_task.need_feedback:
                await self._robot_q.put(
                    PlannerOutput(
                        data=pred_spec_task.request,
                        type=PlannerOutputType.FeedbackRequest,
                    )
                )
                await self._waiting_q.put(HLPTask(goal=task.goal))
            else:
                await self._llm_q.put(HLPTask(goal=task.goal))

        elif isinstance(task, HLPTask):
            pred_hlp_task: HLPTask = self._gen_method.predict(task, self._hlp_processor)
            self._logger.info(
                f"[HLP] {task.goal} {task.feedback.items} -> {pred_hlp_task.subtasks}"
            )
            for llp_task in pred_hlp_task.subtasks:
                await self._llm_q.put(llp_task)

            # if no subtasks
            await self.subtask_done()

        elif isinstance(task, LLPTask):
            # Predict full plan with LLM
            pred_llp_task: LLPTask = self._gen_method.predict(task, self._llp_processor)
            self._logger.info(f"[LLP] {task.goal} -> {pred_llp_task.steps}")

            # Process separated steps
            # for example: move, pick_up, put
            for step in pred_llp_task.steps:
                await self._robot_q.put(
                    PlannerOutput(data=step, type=PlannerOutputType.RobotAction)
                )
            # When subtask is done, add "subtask done"
            await self._robot_q.put(
                PlannerOutput(data=LLPStep("subtask_done"), type=PlannerOutputType.Done)
            )
            await self.subtask_done()
        else:
            raise ValueError("Wrong task type")

    async def subtask_done(self):
        # No other subtasks. Full plan is done.
        if self._llm_q.empty():
            await self._robot_q.put(
                PlannerOutput(data=LLPStep("task_done"), type=PlannerOutputType.Done)
            )

    async def process_output(self, output: PlannerOutput):
        # TODO Process robot actions
        self._logger.info(f"[Processed output] {output}")

    async def process_human_msg(self, msg: HumanMsg):
        # TODO: process human message
        self._logger.info(f"[Human msg] {msg}")


@hydra.main(version_base=None, config_path=".", config_name="sorter_planner")
def get_planner_from_cfg(cfg: SorterPlannerConfig) -> SorterPlanner:
    logger: WandbLogger = instantiate(cfg.logger)
    model: APIModel = instantiate(cfg.model, logger=logger)
    gen_method: FullPlanGeneration = instantiate(
        cfg.gen_method, model=model, logger=logger
    )
    llp_processor: LLPProcessor = instantiate(cfg.llp_processor, logger=logger)
    hlp_processor: HLPProcessor = instantiate(cfg.hlp_processor, logger=logger)
    vc_processor: HLPProcessor = instantiate(cfg.vc_processor, logger=logger)
    spec_processor: SpecProcessor = instantiate(cfg.spec_processor, logger=logger)
    clean_room_processor: CleanRoomProcessor = instantiate(
        cfg.clean_room_processor, logger=logger
    )

    my_planner = SorterPlanner(
        logger=logger,
        llp_processor=llp_processor,
        hlp_processor=hlp_processor,
        vc_processor=vc_processor,
        spec_processor=spec_processor,
        clean_room_processor=clean_room_processor,
        model=model,
        gen_method=gen_method,
    )
    global planner
    planner = my_planner


def get_planner_from_parameters(
    log_dir: Path = Path("outputs/sorter_planner"),
    log_filename: Path = Path("run.log"),
    project_name: str = "llm_sorter",
    run_name: str = "vicuna13b full_plan",
    url: str = "http://127.0.0.1:8080/",
    llp_path_to_prompt_dir: str = "prompts/llp",
    llp_prompt_filename: str = "vicuna_prompt.txt",
    hlp_path_to_prompt_dir: str = "prompts/hlp",
    hlp_prompt_filename: str = "hlp_prompt.txt",
    vc_path_to_prompt_dir: str = "prompts/valid_check",
    vc_prompt_filename: str = "valid_check.txt",
    spec_path_to_prompt_dir: str = "prompt/spec",
    spec_prompt_filename: str = "vicuna_prompt.txt",
) -> SorterPlanner:
    logger: WandbLogger = WandbLogger(
        log_dir=log_dir,
        log_filename=log_filename,
        project_name=project_name,
        run_name=run_name,
        log_to_stdout=True,
    )
    model: APIModel = APIModel(logger=logger, name="vicuna13b docker", url=url)
    gen_method: FullPlanGeneration = FullPlanGeneration(model=model, logger=logger)
    llp_processor: LLPProcessor = LLPProcessor(
        logger,
        path_to_prompt_dir=llp_path_to_prompt_dir,
        prompt_filename=llp_prompt_filename,
        load_prompt_from_file=True,
    )
    hlp_processor: HLPProcessor = HLPProcessor(
        logger=logger,
        path_to_prompt_dir=hlp_path_to_prompt_dir,
        prompt_filename=hlp_prompt_filename,
        load_prompt_from_file=True,
    )
    vc_processor: ValidCheckProcessor = ValidCheckProcessor(
        logger=logger,
        path_to_prompt_dir=vc_path_to_prompt_dir,
        prompt_filename=vc_prompt_filename,
    )
    spec_processor: SpecProcessor = SpecProcessor(
        logger=logger,
        path_to_prompt_dir=spec_path_to_prompt_dir,
        prompt_filename=spec_prompt_filename,
    )
    planner = SorterPlanner(
        logger=logger,
        llp_processor=llp_processor,
        hlp_processor=hlp_processor,
        vc_processor=vc_processor,
        spec_processor=spec_processor,
        model=model,
        gen_method=gen_method,
    )
    return planner


if __name__ == "__main__":
    # planner: SorterPlanner = get_planner_from_parameters()
    planner: SorterPlanner = None
    get_planner_from_cfg()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        asyncio.ensure_future(planner.do_planning())
        yield
        pass

    app = FastAPI(lifespan=lifespan)

    class PostTask(BaseModel):
        goal: str

    class PostFeedback(BaseModel):
        feedback: List[str]

    @app.post("/add_task")
    async def add_task(task: PostTask):
        await planner.add_task(goal=task.goal)

    @app.post("/add_feedback")
    async def add_feedback(feedback: PostFeedback):
        await planner.add_feedback(feedback=feedback.feedback)

    @app.post("/add_llp_task")
    async def add_llp_task(task: PostTask):
        await planner.add_llp_task(goal=task.goal)

    @app.post("/reset")
    async def reset():
        planner.reset()

    @app.post("/valid_check")
    async def valid_check(task: PostTask) -> bool:
        valid_check = await planner.valid_check(goal=task.goal)
        return valid_check

    @app.get("/get_planner_output")
    async def get_planner_output():
        return await planner.get_planner_output()

    uvicorn.run(app, host="0.0.0.0", port=8082)
