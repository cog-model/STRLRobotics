from pathlib import Path
import re
from typing import List, Optional, Tuple
from llm_sorter.models import BaseModelInput, BaseModelOutput
from llm_sorter.processors import BaseProcessor
from llm_sorter.processors import Conversation
from llm_sorter import WandbLogger
from llm_sorter.datasets import AlfredTask
from llm_sorter.datasets.alfred import AlfredTaskType


class AlfredTaskCheckerProcessor(BaseProcessor):
    _TERMINATING_STRING = "done"

    def __init__(
        self,
        logger: WandbLogger,
        path_to_prompt_dir: str | None = None,
        prompt_filename: str | None = None,
        load_prompt_from_file: bool = False,
        **kwargs,
    ) -> None:
        super().__init__(logger=logger, **kwargs)
        self._conversation = Conversation(
            name="vicuna_v1.1",
            system_message="A chat between a curious user and an artificial intelligence assistant. "
            "The assistant gives helpful, detailed, and polite answers to the user's questions.",
            roles=("USER", "ASSISTANT"),
            sep=" ",
            sep2="</s>",
        )

        if load_prompt_from_file:
            path_to_prompt = Path(path_to_prompt_dir) / Path(prompt_filename)
            if path_to_prompt.exists():
                self.build_system_prompt(path_to_prompt=path_to_prompt)

    def build_system_prompt(
        self,
        example_tasks: Optional[List[AlfredTaskType]] = None,
        path_to_prompt: Path = None,
    ) -> str:
        if example_tasks and path_to_prompt:
            raise ValueError(
                "Select one way to create prompt: from exaples or from file. Got both."
            )
        elif example_tasks is not None:
            # setting system prompt
            self._logger.info("Building system prompt from examples...")

            # general description
            user_input = "Imagine, you are an intelligent assistant that helps me "
            user_input += "determine the type of task for a given goal. "
            user_input += f"There are only 7 types of tasks in total: {', '.join(self.task_types_to_prompt())}. "
            user_input += "I tell you the goal of the task, and you only output the task type that best fits. "
            user_input += 'If you get it, reply with the message "Waiting for next input." Understood?'
            assistant_output = "Understood. Waiting for next input."
            self._conversation.append_message(self._conversation.roles[0], user_input)
            self._conversation.append_message(
                self._conversation.roles[1], assistant_output
            )

            # examples
            for task in example_tasks:
                user_input, assistant_output = self._task_to_prompt(task)
                self._conversation.append_message(self._conversation.roles[0], user_input)
                self._conversation.append_message(
                    self._conversation.roles[1], assistant_output
                )
        elif path_to_prompt is not None:
            self._logger.info("Building system prompt from file...")

            with open(path_to_prompt, "r") as file:
                prompt = file.read()
                self._conversation.set_system_message(prompt)

        # Add dummy tasks for tasks
        self._conversation.append_message(self._conversation.roles[0], "{task}.")
        self._conversation.append_message(self._conversation.roles[1], None)

        self._stop_step_pattern = re.compile(r'(\s*\d+\.\s*)(\w+\(("[\w -]+"(,\s)?)*\))*')
        self._logger.info("\n" + self._conversation.get_prompt() + "\n")
        return self._conversation.get_prompt()

    @property
    def system_prompt_is_empty(self) -> bool:
        return len(self._conversation.get_prompt()) == 0

    def task_types_to_prompt(self):
        return [
            task_type.value
            for task_type in AlfredTaskType
            if task_type is not AlfredTaskType.not_set
        ]

    def _task_to_prompt(self, task: AlfredTask) -> Tuple[str, str]:
        user_input = self._goal_to_query(task)
        assistant_output = task.task_type.value
        return user_input, assistant_output

    def to_inputs(
        self,
        task: AlfredTask,
    ) -> BaseModelInput:
        if self.system_prompt_is_empty:
            raise ValueError("System prompt is empty. You need to set system prompt.")
        else:
            self._conversation.update_task_goal(self._goal_to_query(task))
            return BaseModelInput(text=self._conversation.get_prompt())

    def _goal_to_query(self, task: AlfredTask) -> str:
        user_input = f"What is a task type for '{task.goal.capitalize()}'?"
        return user_input

    def to_task(self, task: BaseModelOutput) -> AlfredTask:
        try:
            task_type = AlfredTaskType(task.text.strip())
        except ValueError as e:
            self._logger.info(e)
            task_type = AlfredTaskType.not_set

        return AlfredTask(task_type=task_type)
