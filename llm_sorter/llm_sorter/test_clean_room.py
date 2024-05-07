# import random
import time
import hydra

from hydra.core.config_store import ConfigStore
from hydra.utils import instantiate

from llm_sorter import WandbLogger
from llm_sorter.datasets import HLPDataset, CleanRoomTask
from llm_sorter.gen_methods import BasePlanGeneration
from llm_sorter.infrastructure import TestCleanRoomConfig
from llm_sorter.metrics import BaseTaskMetrics
from llm_sorter.models import BaseLLMModel
from llm_sorter.processors import BaseProcessor


cs = ConfigStore.instance()
cs.store(name="valid_check", node=TestCleanRoomConfig)


@hydra.main(version_base=None, config_path=".", config_name="valid_check")
def test_clean_room(cfg: TestCleanRoomConfig) -> None:
    logger: WandbLogger = instantiate(cfg.logger)
    dataset: HLPDataset = instantiate(cfg.dataset, logger=logger)
    processor: BaseProcessor = instantiate(cfg.processor, logger=logger)
    if not cfg.processor.load_prompt_from_file:
        processor.build_system_prompt([dataset[i] for i in [629, 12, 208]])
    model: BaseLLMModel = instantiate(cfg.model, logger=logger)
    metrics: BaseTaskMetrics = instantiate(
        cfg.metrics, processor=processor, logger=logger
    )
    gen_method: BasePlanGeneration = instantiate(
        cfg.gen_method, model=model, logger=logger
    )

    for i, gt_task in enumerate(dataset):
        try:
            start = time.time()
            predicted_task: CleanRoomTask = gen_method.predict(
                gt_task, processor=processor
            )
            end = time.time()
            d = end - start
        except Exception as e:
            curr_metrics = metrics.update_error()

            logger.info(f"{i + 1}\\{len(dataset)}. Plan id:\t{gt_task.plan_id}")
            logger.info(f"Goal:           {gt_task.goal}")
            logger.info(f"Metrics:        {curr_metrics}")
            logger.info(f"Exception:      {e}\n\n")
        else:
            elapsed_time = metrics.update_time(d)
            curr_metrics = metrics.update(
                predicted_task=predicted_task, target_task=gt_task
            )

            # if True:
            if curr_metrics["Acc"] != 1.0:
                logger.info(f"{i + 1}\\{len(dataset)}. Plan id:\t{gt_task.plan_id}")
                logger.info(f"Goal:           {gt_task.goal}")
                logger.info(f"GT validation:  {gt_task.clean_room}")
                logger.info(f"Predicted plan: {predicted_task.clean_room}")
                logger.info(f"Generation_time:{elapsed_time}")
                logger.info(f"Metrics:        {curr_metrics}\n\n")
            else:
                logger.info(f"{i + 1}\\{len(dataset)}. Plan id:\t{gt_task.plan_id}")

    # total_metrics = {key: f'{value:0.3f}' for key, value in total_metrics.items()}
    total_metrics = metrics.calculate_metrics()

    logger.info(f"Total_metrics:  {total_metrics}")
    logger.wandb_log(total_metrics)
    logger.info("Done.")

    return None


if __name__ == "__main__":
    test_clean_room()
