from pathlib import Path
import re
from typing import List, Optional, Tuple
from llm_sorter.models import BaseModelInput, BaseModelOutput
from llm_sorter.processors import BaseProcessor
from llm_sorter.processors import Conversation
from llm_sorter import WandbLogger
from llm_sorter.datasets import HLPTask, LLPTask


class HLPProcessor(BaseProcessor):
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
        example_tasks: Optional[List[HLPTask]] = None,
        path_to_prompt: Path = None,
    ) -> str:
        if example_tasks and path_to_prompt:
            raise ValueError(
                "Select one way to create prompt: from exaples or from file. Got both."
            )
        elif example_tasks is not None:
            self._logger.info("Building system prompt from examples...")

            user_input = "Imagine, you are an intelligent assistant that helps me divide household "
            user_input += "tasks into subgoals, which will be executed by the home robot step-by-step. "
            user_input += "To clarify the task, I will provide a list of objects that are used in it. "
            user_input += 'If you get it, reply with the message "Waiting for next input." Understood?'
            assistant_output = "Understood. Waiting for next input."
            self._conversation.append_message(self._conversation.roles[0], user_input)
            self._conversation.append_message(
                self._conversation.roles[1], assistant_output
            )

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

    def _items_to_text(self, task: HLPTask):
        return f"List of items to use in the task: {', '.join(task.feedback.items)}"

    def _low_level_tasks_to_text(self, subtasks: List[LLPTask]) -> str:
        return (
            "\n".join(
                f"{i}. {task.goal.capitalize()}."
                for i, task in enumerate(subtasks, start=1)
            )
            + "\n"
            + f"{len(subtasks) + 1}. Done."
        )

    def _task_to_prompt(self, task: HLPTask) -> Tuple[str, str]:
        user_input = self._goal_to_query(task)
        assistant_output = self._low_level_tasks_to_text(task.subtasks)
        task.text = assistant_output
        return user_input, assistant_output

    def to_inputs(
        self,
        task: HLPTask,
    ) -> BaseModelInput:
        if self.system_prompt_is_empty:
            raise ValueError("System prompt is empty. You need to set system prompt.")
        else:
            self._conversation.update_task_goal(self._goal_to_query(task))
            return BaseModelInput(text=self._conversation.get_prompt())

    def _goal_to_query(self, task: HLPTask) -> str:
        user_input = f"{task.goal.capitalize()}."
        user_input += " " + self._items_to_text(task)
        return user_input

    def to_task(self, task: BaseModelOutput) -> HLPTask:
        text: str = task.text.strip().lower()
        goals: List[str] = text.split("\n")
        subtasks: List[LLPTask] = []

        success: bool = False
        for goal in goals:
            string_to_find = r"\d."
            if len(text):
                match = re.search(rf"{string_to_find}\s+([\w\s\d]+)", goal)
                if match:
                    first_word_after_string = match.group(1)
                    # If terminating string is not found,
                    # it's assumed that the model output is
                    # in wrong format, so we return an empty list
                    if first_word_after_string == "done":
                        success = True
                        break
                    subtasks.append(LLPTask(goal=first_word_after_string))

        if not success:
            subtasks = []

        return HLPTask(subtasks=subtasks, text=text)
