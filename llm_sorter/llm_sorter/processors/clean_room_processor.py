from pathlib import Path
import re
from typing import List, Optional, Tuple
from llm_sorter.models import BaseModelInput, BaseModelOutput
from llm_sorter.processors import BaseProcessor, Conversation
from llm_sorter import WandbLogger
from llm_sorter.datasets import CleanRoomTask


class CleanRoomProcessor(BaseProcessor):
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
        example_tasks: Optional[List[CleanRoomTask]] = None,
        path_to_prompt: Path = None,
    ) -> str:
        if example_tasks and path_to_prompt:
            raise ValueError(
                "Select one way to create prompt: from exaples or from file. Got both."
            )
        elif example_tasks is not None:
            self._logger.info("Building system prompt from examples...")

            # setting task
            user_input = """ Given the following input, determine whether it implies the instruction 'Clean the room.' Given a set of instructions, determine if the task specified is cleaning the entire room and all its components rather than focusing on specific items. The goal is to identify instructions that explicitly mention cleaning the room comprehensively, without singling out particular objects. Respond with a simple 'Yes' if the input signifies the cleaning task or 'No' if it does not. Please provide clear and concise answers. """
            self._conversation.set_system_message(
                self._conversation.system_message + user_input
            )
            # user_input = """Given a statement or instruction, determine whether it implies the action of thoroughly cleaning an entire room. Respond with 'Yes' if the input suggests the comprehensive cleaning of the entire room and 'No' otherwise. Avoid specific items in the prompt and focus on the general concept of cleaning the entire room."""
            # user_input += "who can determine if a given task is doable by a home robot. "
            # user_input += "You have a home robot equipped with three actions: move_to(object, location), pick(object, location), and put(object, location). The robot is capable of navigating within a defined area, picking up objects at specific locations, and placing them at other locations. "
            # user_input += "I will tell you a description of the task "
            # user_input += "and you will answer whether it is feasible or not."
            # user_input += 'If you get it, reply with the message "Waiting for next input." Understood?'
            # assistant_output = "Understood. Waiting for next input."

            # self._conversation.append_message(self._conversation.roles[0], user_input)
            # self._conversation.append_message(
            #     self._conversation.roles[1], assistant_output
            # )

            # Add examples
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

    def get_answer(self, task: CleanRoomTask):
        if task.clean_room:
            return "Yes"
        else:
            return "No"

    def _task_to_prompt(self, task: CleanRoomTask) -> Tuple[str, str]:
        user_input = self._goal_to_query(task)
        assistant_output = self.get_answer(task)
        return user_input, assistant_output

    def to_inputs(
        self,
        task: CleanRoomTask,
    ) -> BaseModelInput:
        """Converts task to model input"""
        if self.system_prompt_is_empty:
            raise ValueError("System prompt is empty. You need to set system prompt.")
        else:
            self._conversation.update_task_goal(self._goal_to_query(task))
            return BaseModelInput(text=self._conversation.get_prompt())

    def _goal_to_query(self, task: CleanRoomTask) -> str:
        """Converts task to user input"""
        user_input = f'Input: "{task.goal.lower()}."'
        return user_input

    def to_task(self, task: BaseModelOutput) -> CleanRoomTask:
        """Converts model output to task"""
        text = task.text.lower().replace(".", "").strip()
        # print(task.text)
        if text == "yes":
            clean_room = True
        elif text == "no":
            clean_room = False
        else:
            print(text)
            clean_room = False
        return CleanRoomTask(clean_room=clean_room)
