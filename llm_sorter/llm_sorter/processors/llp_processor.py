import re
from pathlib import Path
from typing import List, Optional, Tuple, Union

from llm_sorter import WandbLogger
from llm_sorter.datasets import LLPDataset, LLPStep, LLPTask
from llm_sorter.models import BaseModelInput, BaseModelOutput
from llm_sorter.processors import BaseProcessor
from llm_sorter.processors import Conversation


class LLPProcessor(BaseProcessor):
    TERMINATING_STRING = "done()"

    @property
    def system_prompt_is_empty(self) -> bool:
        return len(self._conversation.get_prompt()) == 0

    def is_terminating(self, step: LLPStep) -> bool:
        return step.text == self.TERMINATING_STRING

    def __init__(
        self,
        logger: WandbLogger,
        path_to_prompt_dir: Optional[str] = None,
        prompt_filename: Optional[str] = None,
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
        self._stop_pattern = re.compile(rf"\d+\. {self.TERMINATING_STRING}.")
        if load_prompt_from_file:
            path_to_prompt = Path(path_to_prompt_dir) / Path(prompt_filename)
            if path_to_prompt.exists():
                self.build_system_prompt(path_to_prompt=path_to_prompt)

    def build_system_prompt(
        self,
        example_tasks: Optional[List[LLPTask]] = None,
        path_to_prompt: Path = None,
    ) -> str:
        if example_tasks and path_to_prompt:
            raise ValueError(
                "Select one way to create prompt: from exaples or from file. Got both."
            )
        elif example_tasks is not None:
            self._logger.info("Building system prompt from examples...")

            user_input = " Imagine, you are a robot operating in a house. "
            user_input += "I can ask you to do various tasks and "
            user_input += "you're gonna tell me the sequence of actions you would do to accomplish your task. "
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
        self._conversation.append_message(
            self._conversation.roles[0], "How would you {task}?"
        )
        self._conversation.append_message(self._conversation.roles[1], None)

        self._stop_step_pattern = re.compile(r'(\s*\d+\.\s*)(\w+\(("[\w -]+"(,\s)?)*\))*')
        self._logger.info("\n" + self._conversation.get_prompt() + "\n")

    def _goal_to_query(self, goal: str) -> str:
        user_input = f"How would you {goal.lower()}?"
        return user_input

    def _step_to_text(self, step: LLPStep) -> str:
        """Transform step class to text:
        move: ['cup', 'table'] -> move('cup', 'table')
        """
        arguments = [f'"{argument}"' for argument in step.arguments]
        text = f'{step.action}({", ".join(arguments)})'
        return text

    def _steps_to_text(
        self, steps: List[LLPStep], add_terminating_string: bool = True
    ) -> str:
        """Transform list of steps (ground true steps) to the one row (plan)"""
        text = ", ".join(
            [
                f"{step_idx}. {self._step_to_text(step)}"
                for step_idx, step in enumerate(steps, start=1)
            ]
        )
        if add_terminating_string:
            text += f", {len(steps) + 1}. {self.TERMINATING_STRING}."
        return text

    def _task_to_prompt(self, task: LLPTask) -> Tuple[str, str]:
        user_input = self._goal_to_query(task.goal)
        assistant_output = self._steps_to_text(task.steps)
        task.text = assistant_output
        return user_input, assistant_output

    def to_inputs(
        self,
        task: LLPTask,
    ) -> BaseModelInput:
        if self.system_prompt_is_empty:
            raise ValueError("System prompt is empty. You need to set system prompt.")
        else:
            self._conversation.update_task_goal(self._goal_to_query(task.goal))
            return BaseModelInput(text=self._conversation.get_prompt())

    def _text_to_steps(
        self, task_text: str, cut_one_step: bool = False
    ) -> Union[List[LLPStep], LLPStep, None]:
        if cut_one_step:
            stop_match = self._stop_step_pattern.match(task_text)
            if stop_match is None:
                return None
            else:
                return self._parse_action(stop_match.group(2))
        else:
            stop_match = self._stop_step_pattern.findall(task_text)
            steps = []
            if stop_match is None:
                return steps
            else:
                for i in range(len(stop_match) - 1):
                    step_text = stop_match[i][1]
                    step = self._parse_action(step_text)
                    if step is not None:
                        steps.append(step)
                return steps

    def _parse_action(self, step_text: str) -> Optional[LLPStep]:
        """Parse action with arguments to step.
        text: put_on('pepper', 'white box')
        action: put_on
        arguments: ['pepper', 'white box']
        """
        step_decomposition_pattern = re.compile(r"\s*([A-Za-z_][A-Za-z_\s\d-]+)")
        arguments = step_decomposition_pattern.findall(step_text)

        if arguments is None:
            return None
        if len(arguments) == 1:
            step = LLPStep(text=step_text)
        else:
            step = LLPStep(action=arguments[0], arguments=arguments[1:], text=step_text)
            return step

    def to_task(self, task: Union[BaseModelOutput, List[LLPStep]]) -> LLPTask:
        # Autoregressive Mode
        if isinstance(task, list):
            return LLPStep(text=self._steps_to_text(task), steps=task)

        # Full plan generation mode
        task.text = task.text.replace("\\", "")
        task.text = task.text.lower()

        stop_match = self._stop_pattern.search(task.text)

        if stop_match is not None:
            task.text = task.text[: stop_match.end() + 2].strip(" \n\t")
        else:
            task.text = task.text.strip(" \n\t")

        steps = self._text_to_steps(task_text=task.text)

        return LLPTask(text=task.text, steps=steps)


if __name__ == "__main__":
    logger = WandbLogger()
    dataset = LLPDataset(
        logger, path_to_data_dir="data/llp/", dataset_filename="first200"
    )

    processor = LLPProcessor(logger=logger)
    processor.build_system_prompt([dataset[0], dataset[3]])

    print(processor.to_inputs(dataset[1]).text)
