from pathlib import Path
import re
from typing import List, Optional
from llm_sorter.models import BaseModelInput, BaseModelOutput
from llm_sorter.processors import BaseProcessor
from llm_sorter.processors import Conversation
from llm_sorter import WandbLogger
from llm_sorter.datasets import SpecTask
import inflect


class SpecProcessor(BaseProcessor):
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
        self._p = inflect.engine()
        if load_prompt_from_file:
            path_to_prompt = Path(path_to_prompt_dir) / Path(prompt_filename)
            if path_to_prompt.exists():
                self.build_system_prompt(path_to_prompt=path_to_prompt)

    def build_system_prompt(
        self,
        example_tasks: Optional[List[SpecTask]] = None,
        path_to_prompt: Path = None,
    ) -> str:
        if example_tasks and path_to_prompt:
            raise ValueError(
                "Select one way to create prompt: from exaples or from file. Got both."
            )
        elif example_tasks is not None:
            self._logger.info("Building system prompt from examples...")

            user_input = (
                "Imagine, you are an intelligent assistant who helps a household robot "
            )
            user_input += "to clarify a task. If the list of objects to work with is not clear in the task, "
            user_input += (
                "e.g. all toys are specified, you should ask for a list of all toys. "
            )
            user_input += "You don't need to ask for individual objects or furniture. "
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

        self._logger.info("\n" + self._conversation.get_prompt() + "\n")
        return self._conversation.get_prompt()

    @property
    def system_prompt_is_empty(self) -> bool:
        return len(self._conversation.get_prompt()) == 0

    def _goal_to_query(self, task: SpecTask):
        text: str = f"Task: {task.goal}"
        return text

    def _task_to_prompt(self, task: SpecTask):
        user_input = self._goal_to_query(task)
        assistant_output = f"Request: {task.request}"
        return user_input, assistant_output

    def to_inputs(self, task: SpecTask) -> BaseModelInput:
        if self.system_prompt_is_empty:
            raise ValueError("System prompt is empty. You need to set system prompt.")
        else:
            self._conversation.update_task_goal(self._goal_to_query(task))
            return BaseModelInput(text=self._conversation.get_prompt())

    def to_task(self, task: BaseModelOutput) -> SpecTask:
        text = task.text
        pattern = r"\s*Request: (.+)"  # This regex pattern captures everything after "Request: "

        match = re.match(pattern, text)

        if match:
            request = match.group(1).strip().lower()
            # remove multiple items
            if len(request.split(", ")) > 1:
                request = "none"
            # remove singular objects
            elif not self._p.singular_noun(request):
                request = "none"
        else:
            request = "none"

        return SpecTask(request=request)
