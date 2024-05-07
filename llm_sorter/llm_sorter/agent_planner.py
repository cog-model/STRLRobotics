# TODO:
# - change all sorter_planner copied configs \ deps -> agent_planner
# - remove human msg

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Union, Optional
from contextlib import asynccontextmanager

import hydra
from hydra.core.config_store import ConfigStore
from hydra.utils import instantiate
from pydantic import BaseModel

from llm_sorter.models import APIModel, BaseLLMModel
from llm_sorter import WandbLogger
from llm_sorter.datasets import LLPTask, HLPTask, LLPStep, BaseTask, VCTask
from llm_sorter.infrastructure import AgentPlannerConfig
from llm_sorter.gen_methods import BasePlanGeneration, FullPlanGeneration
from llm_sorter.processors import (
    BaseProcessor,
    LLPProcessor,
    HLPProcessor,
    ValidCheckProcessor,
)
from asyncio import Queue
import asyncio
from fastapi import FastAPI
import uvicorn


cs = ConfigStore.instance()
cs.store(name="sorter_planner", node=AgentPlannerConfig)


class PlannerOutputType(Enum):
    RobotAction = "RobotAction"
    FeedbackRequest = "FeedbackRequest"
    UnvalidTask = "UnvalidTask"
    Done = "Done"


class HumanMsgType(Enum):
    UnvalidTask = "UnvalidTask"
    PlannerIsBusy = "PlannerIsBusy"


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


class AgentPlanner:
    def __init__(
        self,
        logger: WandbLogger,
        llp_processor: LLPProcessor,
        hlp_processor: HLPProcessor,
        vc_processor: ValidCheckProcessor,
        model: BaseLLMModel,
        gen_method: FullPlanGeneration,
        **kwargs,
    ) -> None:
        self._logger: WandbLogger = logger
        self._model: BaseLLMModel = model

        self._llp_processor: BaseProcessor = llp_processor
        self._hlp_processor: BaseProcessor = hlp_processor
        self._vc_processor: BaseProcessor = vc_processor
        self._gen_method: BasePlanGeneration = gen_method

        self._robot_q: Queue[LLPStep] = Queue()
        self._human_q: Queue[HumanMsg] = Queue()
        self._llm_q: Queue[BaseTask] = Queue()

    async def add_task(self, goal: str) -> None:
        if not self._llm_q.empty() or not self._robot_q.empty():
            msg = HumanMsg(data="Planner is busy", type=HumanMsgType.PlannerIsBusy)
            self._human_q.put_nowait(msg)
        else:
            await self._llm_q.put(VCTask(goal=goal))
            self._logger.info(f"[Added HLP task] {goal}")

    async def do_planning(self):
        while True:
            while not self._robot_q.empty():
                output = self._robot_q.get_nowait()
                await self.process_output(output)

            if not self._llm_q.empty():
                task = await self._llm_q.get()
                await self.process_task(task)

            while not self._human_q.empty():
                msg = await self._human_q.get()
                await self.process_human_msg(msg)

            await asyncio.sleep(0.1)

    async def add_llp_task(self, goal: str) -> None:
        if not self._llm_q.empty() or not self._robot_q.empty():
            msg = HumanMsg(data="Planner is busy", type=HumanMsgType.PlannerIsBusy)
            await self._human_q.put(msg)
        else:
            await self._llm_q.put(LLPTask(goal=goal))
            self._logger.info(f"[Added LLP task] {goal}")

    async def process_task(self, task: Union[VCTask, HLPTask, LLPTask]):
        """Process a given task based on its type with the LLM model.

        Args:
            task (Union[VCTask, HLPTask, LLPTask]): The task to be processed.
        Raises:
            ValueError: If the task type is not recognized.
        """
        if isinstance(task, VCTask):
            pred_vc_task: VCTask = self._gen_method.predict(task, self._vc_processor)
            self._logger.info(f"[VC] {task.goal} -> {pred_vc_task.valid_plan}")

            if pred_vc_task.valid_plan:
                await self._llm_q.put(HLPTask(goal=task.goal))
            else:
                await self._robot_actions_q.put(
                    PlannerOutput(data="Unvalid task", type=PlannerOutputType.UnvalidTask)
                )
        elif isinstance(task, HLPTask):
            pred_hlp_task: HLPTask = self._gen_method.predict(task, self._hlp_processor)
            self._logger.info(f"[HLP] {task.goal} -> {pred_hlp_task.subtasks}")

            for llp_task in pred_hlp_task.subtasks:
                await self._llm_q.put(llp_task)
        elif isinstance(task, LLPTask):
            pred_llp_task: LLPTask = self._gen_method.predict(task, self._llp_processor)
            self._logger.info(f"[LLP] {task.goal} -> {pred_llp_task.steps}")

            for step in pred_llp_task.steps:
                await self._robot_q.put(
                    PlannerOutput(data=step, type=PlannerOutputType.RobotAction)
                )
        else:
            raise ValueError("Wrong task type")

    async def process_output(self, output: PlannerOutput):
        # TODO Process robot actions
        self._logger.info(f"[Processed output] {output}")

    async def process_human_msg(self, msg: HumanMsg):
        # TODO: process human message
        self._logger.info(f"[Human msg] {msg}")


@hydra.main(version_base=None, config_path=".", config_name="sorter_planner")
def get_planner_from_cfg(cfg: AgentPlannerConfig) -> AgentPlanner:
    logger: WandbLogger = instantiate(cfg.logger)
    model: APIModel = instantiate(cfg.model, logger=logger)
    gen_method: FullPlanGeneration = instantiate(
        cfg.gen_method, model=model, logger=logger
    )
    llp_processor: LLPProcessor = instantiate(cfg.llp_processor, logger=logger)
    hlp_processor: HLPProcessor = instantiate(cfg.hlp_processor, logger=logger)
    vc_processor: HLPProcessor = instantiate(cfg.vc_processor, logger=logger)

    my_planner = AgentPlanner(
        logger=logger,
        llp_processor=llp_processor,
        hlp_processor=hlp_processor,
        vc_processor=vc_processor,
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
) -> AgentPlanner:
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
    planner = AgentPlanner(
        logger=logger,
        llp_processor=llp_processor,
        hlp_processor=hlp_processor,
        vc_processor=vc_processor,
        model=model,
        gen_method=gen_method,
    )
    return planner


if __name__ == "__main__":
    # planner: AgentPlanner = get_planner_from_parameters()
    planner: AgentPlanner = None
    get_planner_from_cfg()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        asyncio.ensure_future(planner.do_planning())
        yield
        pass

    app = FastAPI(lifespan=lifespan)

    class PostTask(BaseModel):
        task_type: Optional[str]
        goal: Optional[str]
        img: Optional[list]
        classes: Optional[tuple]
        cur_subtask_ptr: Optional[int]
        subtask_to_return_to: Optional[str]
        subtask: Optional[str]

    @app.post("/add_task")
    async def add_task(task: PostTask):
        await planner.add_task(goal=task.goal)

    @app.post("/add_llp_task")
    async def add_llp_task(task: PostTask):
        await planner.add_llp_task(goal=task.goal)

    uvicorn.run(app, host="0.0.0.0", port=8083)
