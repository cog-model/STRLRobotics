import time

import hydra
from hydra.core.config_store import ConfigStore
from hydra.utils import instantiate

from llm_sorter import WandbLogger
from llm_sorter.datasets import AlfredDataset
from llm_sorter.gen_methods import BasePlanGeneration
from llm_sorter.infrastructure import TestAlfredTaskTypeConfig
from llm_sorter.metrics import BaseTaskMetrics
from llm_sorter.models import BaseLLMModel
from llm_sorter.processors import BaseProcessor


cs = ConfigStore.instance()
cs.store(name="alfred_planner", node=TestAlfredTaskTypeConfig)


@hydra.main(version_base=None, config_path=".", config_name="alfred_planner")
def test_alfred_planner(cfg: TestAlfredTaskTypeConfig) -> None:
    logger: WandbLogger = instantiate(cfg.logger)
    dataset: AlfredDataset = instantiate(cfg.dataset, logger=logger)
    # prompt_dataset: AlfredDataset = instantiate(cfg.prompt_dataset, logger=logger)
    processor: BaseProcessor = instantiate(cfg.processor, logger=logger)
    if not cfg.processor.load_prompt_from_file:
        processor.build_system_prompt(
            [dataset[i] for i in [616, 650, 679, 700, 707, 727, 783, 429, 362, 11, 21]]
        )
    model: BaseLLMModel = instantiate(cfg.model, logger=logger)
    metrics: BaseTaskMetrics = instantiate(
        cfg.metrics, processor=processor, logger=logger
    )
    gen_method: BasePlanGeneration = instantiate(
        cfg.gen_method, model=model, logger=logger
    )

    for i, gt_task in enumerate(dataset):
        # gt_task.goal = 'remove the red apple from the table in the drawer'

        try:
            start = time.time()
            predicted_task = gen_method.predict(gt_task, processor=processor)
            end = time.time()
            d = end - start
        except Exception as e:
            curr_metrics = metrics.update_error()

            logger.info(f"{i + 1}\\{len(dataset)}. Plan id:\t{gt_task.plan_id}")
            logger.info(f"Goal:           {gt_task.goal}")
            logger.info(f"GT task type:   {gt_task.task_type.value}")
            logger.info(f"Metrics:        {curr_metrics}")
            logger.info(f"Exception:      {e}\n\n")
        else:
            elapsed_time = metrics.update_time(d)
            curr_metrics = metrics.update(
                predicted_task=predicted_task, target_task=gt_task
            )

            # Log fails only
            if curr_metrics["PEM"] != 1.0:
                logger.info(f"{i + 1}\\{len(dataset)}. Plan id:\t{gt_task.plan_id}")
                logger.info(f"Goal:           {gt_task.goal}")
                logger.info(f"GT task type:   {gt_task.task_type.value}")
                logger.info(f"Predicted plan: {predicted_task.task_type.value}")
                logger.info(f"Generation_time:{elapsed_time}")
                logger.info(f"Metrics:        {curr_metrics}\n\n")
            else:
                logger.info(f"{i + 1}\\{len(dataset)}. Plan id:\t{gt_task.plan_id}")

    total_metrics = metrics.calculate_metrics()
    total_metrics = {key: f"{value:0.3f}" for key, value in total_metrics.items()}

    logger.info(f"Total_metrics:  {total_metrics}")
    logger.wandb_log(total_metrics)
    logger.info("Done.")

    return None


if __name__ == "__main__":
    test_alfred_planner()
