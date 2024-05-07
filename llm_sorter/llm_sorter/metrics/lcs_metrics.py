from typing import Any, Callable, Dict, List

import numpy as np

from llm_sorter import WandbLogger
from llm_sorter.datasets import BaseTask
from llm_sorter.datasets import HLPTask, AlfredTask, SpecTask
from llm_sorter.metrics import BaseMetric, BaseTaskMetrics, preprocess
from llm_sorter.processors import BaseProcessor


class LCSA(BaseMetric):
    """Longest common subarray"""

    def __init__(
        self,
        pred_process_f: Callable,
        target_process_f: Callable,
        name: str = "LCSA",
        **kwargs,
    ):
        super().__init__(pred_process_f, target_process_f, name, **kwargs)

    @preprocess
    def __call__(self, pred: List, target: List) -> float:
        p, t = len(pred), len(target)

        arr = [[0 for _ in range(t + 1)] for _ in range(p + 1)]
        mx = 0
        for i in range(p + 1):
            for j in range(t + 1):
                if i == 0 or j == 0:
                    arr[i][j] = 0
                elif pred[i - 1] == target[j - 1]:
                    arr[i][j] = arr[i - 1][j - 1] + 1
                    mx = max(mx, arr[i][j])
                else:
                    arr[i][j] = 0
        return mx / t


class LCSS(BaseMetric):
    """Longest common subsequence"""

    def __init__(
        self,
        pred_process_f: Callable,
        target_process_f: Callable,
        name: str = "LCSS",
        normalize_both: bool = False,
        **kwargs,
    ):
        super().__init__(pred_process_f, target_process_f, name, **kwargs)
        self._normalize_both = normalize_both

    @preprocess
    def __call__(self, pred: List, target: List) -> float:
        p = len(pred)
        t = len(target)

        # declaring the array for storing the dp values
        arr = [[None] * (t + 1) for _ in range(p + 1)]

        for i in range(p + 1):
            for j in range(t + 1):
                if i == 0 or j == 0:
                    arr[i][j] = 0
                elif pred[i - 1] == target[j - 1]:
                    arr[i][j] = arr[i - 1][j - 1] + 1
                else:
                    arr[i][j] = max(arr[i - 1][j], arr[i][j - 1])

        if self._normalize_both:
            normalize_coef = max(p, t)
        else:
            normalize_coef = t

        return arr[p][t] / normalize_coef


class EM(LCSS):
    def __init__(
        self,
        pred_process_f: Callable[..., Any],
        target_process_f: Callable[..., Any],
        name: str = "EM",
        normalize_both: bool = True,
        **kwargs,
    ):
        super().__init__(pred_process_f, target_process_f, name, normalize_both, **kwargs)

    def __call__(self, pred: List, target: List) -> float:
        return float(super().__call__(pred, target) == 1)


class LLPMetrics(BaseTaskMetrics):
    def __init__(self, logger: WandbLogger, processor: BaseProcessor, **kwargs):
        super().__init__(logger, processor, **kwargs)
        # TODO: Fix this
        # Add metric functions
        # def to_text(task):
        #     return [task.text]

        def to_action_list(task):
            return [step.action.lower() for step in task.steps]

        def to_step_list(task):
            if len(task.steps):
                task.text = processor._steps_to_text(task.steps)

            task.steps = processor._text_to_steps(task.text)
            return [step.text.lower() for step in task.steps]

        a_lcsq = LCSS(to_action_list, to_action_list, "A-LCSS")
        p_lcsq = LCSS(to_step_list, to_step_list, "P-LCSS")
        a_lcsg = LCSA(to_action_list, to_action_list, "A-LCSA")
        p_lcsg = LCSA(to_step_list, to_step_list, "P-LCSA")
        pem = EM(to_step_list, to_step_list, "PEM", normalize_both=True)
        aem = EM(to_action_list, to_action_list, "AEM", normalize_both=True)
        psem = EM(to_step_list, to_step_list, "PSEM", normalize_both=False)
        asem = EM(to_action_list, to_action_list, "ASEM", normalize_both=False)
        self._times = []
        self._metric_list: List[BaseMetric] = [
            a_lcsq,
            p_lcsq,
            a_lcsg,
            p_lcsg,
            pem,
            aem,
            psem,
            asem,
        ]
        # Dict to save intermediate values
        self._metric_values: Dict = {
            metric_cls.name: 0.0 for metric_cls in self._metric_list
        }
        self._count = 0.0

    def update_time(self, timedelta: float) -> str:
        self._times.append(timedelta)
        return f"{timedelta: 0.3f} sec"

    def update(self, predicted_task: BaseTask, target_task: BaseTask) -> str:
        self._count += 1

        metric_values: Dict = {}

        for metric_class in self._metric_list:
            metric_values[metric_class.name] = metric_class(predicted_task, target_task)

        for name, value in metric_values.items():
            self._metric_values[name] += value

        return metric_values

    def update_error(self):
        self._count += 1
        return "Error. Answer cannot be parsed"

    def calculate_metrics(self) -> Dict:
        total_metrics = {
            key: value / self._count for key, value in self._metric_values.items()
        }
        timedeltas = np.array(self._times)
        mean_time = timedeltas.mean()
        std_time = timedeltas.std()

        total_metrics["mean_time"] = mean_time
        total_metrics["std_time"] = std_time
        return total_metrics


class HLPMetrics(BaseTaskMetrics):
    def __init__(self, logger: WandbLogger, processor: BaseProcessor, **kwargs):
        super().__init__(logger, processor, **kwargs)
        # TODO: Fix this
        # Add metric functions
        # def to_text(task):
        #     return [task.text]

        def to_subtask_text(task: HLPTask) -> List[str]:
            return [subtask.goal.lower() for subtask in task.subtasks]

        p_lcsq = LCSS(to_subtask_text, to_subtask_text, "P-LCSS")
        p_lcsg = LCSA(to_subtask_text, to_subtask_text, "P-LCSA")
        pem = EM(to_subtask_text, to_subtask_text, "PEM", normalize_both=True)
        psem = EM(to_subtask_text, to_subtask_text, "PSEM", normalize_both=False)
        self._times = []
        self._metric_list: List[BaseMetric] = [
            p_lcsq,
            p_lcsg,
            pem,
            psem,
        ]
        # Dict to save intermediate values
        self._metric_values: Dict = {
            metric_cls.name: 0.0 for metric_cls in self._metric_list
        }
        self._count = 0.0

    def update_time(self, timedelta: float) -> str:
        self._times.append(timedelta)
        return f"{timedelta: 0.3f} sec"

    def update(self, predicted_task: BaseTask, target_task: BaseTask) -> str:
        self._count += 1

        metric_values: Dict = {}

        for metric_class in self._metric_list:
            metric_values[metric_class.name] = metric_class(predicted_task, target_task)

        for name, value in metric_values.items():
            self._metric_values[name] += value

        return metric_values

    def update_error(self):
        self._count += 1
        return "Error. Answer cannot be parsed"

    def calculate_metrics(self) -> Dict:
        total_metrics = {
            key: value / self._count for key, value in self._metric_values.items()
        }
        timedeltas = np.array(self._times)
        mean_time = timedeltas.mean()
        std_time = timedeltas.std()

        total_metrics["mean_time"] = mean_time
        total_metrics["std_time"] = std_time
        return total_metrics


class AlfredTaskTypeMetrics(BaseTaskMetrics):
    def __init__(self, logger: WandbLogger, processor: BaseProcessor, **kwargs):
        super().__init__(logger, processor, **kwargs)
        # TODO: Fix this
        # Add metric functions
        # def to_text(task):
        #     return [task.text]

        def to_subtask_text(task: AlfredTask) -> List[str]:
            return [task.task_type.value]

        pem = EM(to_subtask_text, to_subtask_text, "Acc", normalize_both=True)
        self._times = []
        self._metric_list: List[BaseMetric] = [
            pem,
        ]
        # Dict to save intermediate values
        self._metric_values: Dict = {
            metric_cls.name: 0.0 for metric_cls in self._metric_list
        }
        self._count = 0.0

    def update_time(self, timedelta: float) -> str:
        self._times.append(timedelta)
        return f"{timedelta: 0.3f} sec"

    def update(self, predicted_task: BaseTask, target_task: BaseTask) -> str:
        self._count += 1

        metric_values: Dict = {}

        for metric_class in self._metric_list:
            metric_values[metric_class.name] = metric_class(predicted_task, target_task)

        for name, value in metric_values.items():
            self._metric_values[name] += value

        return metric_values

    def update_error(self):
        self._count += 1
        return "Error. Answer cannot be parsed"

    def calculate_metrics(self) -> Dict:
        total_metrics = {
            key: value / self._count for key, value in self._metric_values.items()
        }
        timedeltas = np.array(self._times)
        mean_time = timedeltas.mean()
        std_time = timedeltas.std()

        total_metrics["mean_time"] = mean_time
        total_metrics["std_time"] = std_time
        return total_metrics


class ValidCheckMetrics(BaseTaskMetrics):
    def __init__(self, logger: WandbLogger, processor: BaseProcessor, **kwargs):
        super().__init__(logger, processor, **kwargs)
        # TODO: Fix this
        # Add metric functions
        # def to_text(task):
        #     return [task.text]

        def to_subtask_text(task: HLPTask) -> List[str]:
            return [task.valid_plan]

        pem = EM(to_subtask_text, to_subtask_text, "Acc", normalize_both=True)
        self._times = []
        self._metric_list: List[BaseMetric] = [
            pem,
        ]
        # Dict to save intermediate values
        self._metric_values: Dict = {
            metric_cls.name: 0.0 for metric_cls in self._metric_list
        }
        self._count = 0.0

    def update_time(self, timedelta: float) -> str:
        self._times.append(timedelta)
        return f"{timedelta: 0.3f} sec"

    def update(self, predicted_task: BaseTask, target_task: BaseTask) -> str:
        self._count += 1

        metric_values: Dict = {}

        for metric_class in self._metric_list:
            metric_values[metric_class.name] = metric_class(predicted_task, target_task)

        for name, value in metric_values.items():
            self._metric_values[name] += value

        return metric_values

    def update_error(self):
        self._count += 1
        return "Error. Answer cannot be parsed"

    def calculate_metrics(self) -> Dict:
        total_metrics = {
            key: value / self._count for key, value in self._metric_values.items()
        }
        timedeltas = np.array(self._times)
        mean_time = timedeltas.mean()
        std_time = timedeltas.std()

        total_metrics["mean_time"] = mean_time
        total_metrics["std_time"] = std_time
        return total_metrics


class SpecMetrics(BaseTaskMetrics):
    def __init__(self, logger: WandbLogger, processor: BaseProcessor, **kwargs):
        super().__init__(logger, processor, **kwargs)
        # TODO: Fix this
        # Add metric functions
        # def to_text(task):
        #     return [task.text]

        def to_subtask_text(task: SpecTask) -> List[str]:
            return [task.request]

        pem = EM(to_subtask_text, to_subtask_text, "Acc", normalize_both=True)
        self._times = []
        self._metric_list: List[BaseMetric] = [
            pem,
        ]
        # Dict to save intermediate values
        self._metric_values: Dict = {
            metric_cls.name: 0.0 for metric_cls in self._metric_list
        }
        self._count = 0.0

    def update_time(self, timedelta: float) -> str:
        self._times.append(timedelta)
        return f"{timedelta: 0.3f} sec"

    def update(self, predicted_task: BaseTask, target_task: BaseTask) -> str:
        self._count += 1

        metric_values: Dict = {}

        for metric_class in self._metric_list:
            metric_values[metric_class.name] = metric_class(predicted_task, target_task)

        for name, value in metric_values.items():
            self._metric_values[name] += value

        return metric_values

    def update_error(self):
        self._count += 1
        return "Error. Answer cannot be parsed"

    def calculate_metrics(self) -> Dict:
        total_metrics = {
            key: value / self._count for key, value in self._metric_values.items()
        }
        timedeltas = np.array(self._times)
        mean_time = timedeltas.mean()
        std_time = timedeltas.std()

        total_metrics["mean_time"] = mean_time
        total_metrics["std_time"] = std_time
        return total_metrics


class CleanRoomMetrics(BaseTaskMetrics):
    def __init__(self, logger: WandbLogger, processor: BaseProcessor, **kwargs):
        super().__init__(logger, processor, **kwargs)
        # TODO: Fix this
        # Add metric functions
        # def to_text(task):
        #     return [task.text]

        def to_subtask_text(task: HLPTask) -> List[str]:
            return [task.clean_room]

        pem = EM(to_subtask_text, to_subtask_text, "Acc", normalize_both=True)
        self._times = []
        self._metric_list: List[BaseMetric] = [
            pem,
        ]
        # Dict to save intermediate values
        self._metric_values: Dict = {
            metric_cls.name: 0.0 for metric_cls in self._metric_list
        }
        self._count = 0.0

    def update_time(self, timedelta: float) -> str:
        self._times.append(timedelta)
        return f"{timedelta: 0.3f} sec"

    def update(self, predicted_task: BaseTask, target_task: BaseTask) -> str:
        self._count += 1

        metric_values: Dict = {}

        for metric_class in self._metric_list:
            metric_values[metric_class.name] = metric_class(predicted_task, target_task)

        for name, value in metric_values.items():
            self._metric_values[name] += value

        return metric_values

    def update_error(self):
        self._count += 1
        return "Error. Answer cannot be parsed"

    def calculate_metrics(self) -> Dict:
        total_metrics = {
            key: value / self._count for key, value in self._metric_values.items()
        }
        timedeltas = np.array(self._times)
        mean_time = timedeltas.mean()
        std_time = timedeltas.std()

        total_metrics["mean_time"] = mean_time
        total_metrics["std_time"] = std_time
        return total_metrics
