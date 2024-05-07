from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from omegaconf import MISSING


# -------------------------------------------------
# -- Logger
# -------------------------------------------------


@dataclass
class BaseLoggerConfig:
    _target_ = MISSING


@dataclass
class WandbLoggerConfig(BaseLoggerConfig):
    _target_: str = "llm_sorter.WandbLogger"
    log_filename: str = "run.log"
    log_dir: str = "${hydra:run.dir}/"
    project_name: str = "llm_sorter"
    run_name: str = "${model.name} ${gen_method.name} ${dataset.dataset_filename}"


# -------------------------------------------------
# -- Dataset
# -------------------------------------------------


@dataclass
class BaseDatasetConfig:
    _target_: str = MISSING
    dataset_filename: str = MISSING


@dataclass
class LLPDatasetConfig(BaseDatasetConfig):
    path_to_data_dir: Path = "${experiment.path_to_data_dir}/llp"
    # dataset_filename: str = "first200"
    dataset_filename: str = "simple_plans"
    # dataset_filename: str = "tests"
    _target_: str = "llm_sorter.datasets.LLPDataset"


@dataclass
class LLPTestDatasetConfig(BaseDatasetConfig):
    path_to_data_dir: Path = "${experiment.path_to_data_dir}/llp"
    # dataset_filename: str = "first200"
    # dataset_filename: str = "simple_plans"
    dataset_filename: str = "llp_tests"
    _target_: str = "llm_sorter.datasets.LLPDataset"


@dataclass
class SorterDatasetConfig(BaseDatasetConfig):
    _target_: str = MISSING
    dataset_filename: str = "sorter_planning"


@dataclass
class HLPDatasetConfig(BaseDatasetConfig):
    path_to_data_dir: Path = "${experiment.path_to_data_dir}/hlp"
    # dataset_filename: str = "hlp_tests"
    # dataset_filename: str = "converted_2pp"
    # dataset_filename: str = "converted_simple_plans"
    # dataset_filename: str = "plans_hl_numbered_objects"
    dataset_filename: str = "all"
    _target_: str = "llm_sorter.datasets.HLPDataset"


@dataclass
class ValidCheckDatasetConfig(BaseDatasetConfig):
    path_to_data_dir: Path = "${experiment.path_to_data_dir}/hlp"
    # dataset_filename: str = "all"
    # dataset_filename: str = "valid_check"
    dataset_filename: str = "all_valid"
    _target_: str = "llm_sorter.datasets.HLPDataset"


@dataclass
class CleanRoomDatasetConfig(BaseDatasetConfig):
    path_to_data_dir: Path = "${experiment.path_to_data_dir}/hlp"
    # dataset_filename: str = "clean_room"
    dataset_filename: str = "all_valid"
    # dataset_filename: str = "clean_room"
    _target_: str = "llm_sorter.datasets.HLPDataset"


@dataclass
class AlfredDatasetConfig(BaseDatasetConfig):
    path_to_data_dir: Path = "${experiment.path_to_data_dir}/alfred"
    dataset_filename: str = "valid_unseen_highlevel"
    _target_: str = "llm_sorter.datasets.AlfredDataset"


@dataclass
class AlfredPromptDatasetConfig(BaseDatasetConfig):
    path_to_data_dir: Path = "${experiment.path_to_data_dir}/alfred"
    dataset_filename: str = "valid_seen_highlevel"
    _target_: str = "llm_sorter.datasets.AlfredDataset"


# -------------------------------------------------
# -- Model
# -------------------------------------------------


@dataclass
class BaseModelConfig:
    name: str = MISSING
    _target_: str = MISSING


@dataclass
class APIModelConfig(BaseModelConfig):
    _target_: str = "llm_sorter.models.api_model.APIModel"
    name: str = "docker vicuna"
    url: str = "http://127.0.0.1:8080/"


# -------------------------------------------------
# -- Experiment
# -------------------------------------------------


@dataclass
class BaseExperimentConfig:
    logging_dir: str = "${hydra:run.dir}/"
    path_to_data_dir: str = "${hydra:runtime.cwd}/data/"
    device: int = 1
    seed: int = 0


@dataclass
class LLPExperimentConfig(BaseExperimentConfig):
    logging_dir: str = "${hydra:run.dir}/"
    path_to_data_dir: str = "${hydra:runtime.cwd}/data/"
    device: int = 1
    seed: int = 0


@dataclass
class HLPExperimentConfig(BaseExperimentConfig):
    logging_dir: str = "${hydra:run.dir}/"
    path_to_data_dir: str = "${hydra:runtime.cwd}/data/"
    device: int = 1
    seed: int = 0


@dataclass
class SpecExperimentConfig(BaseExperimentConfig):
    logging_dir: str = "${hydra:run.dir}/"
    path_to_data_dir: str = "${hydra:runtime.cwd}/data/"
    device: int = 1


@dataclass
class AlfredExperimentConfig(BaseExperimentConfig):
    logging_dir: str = "${hydra:run.dir}/"
    path_to_data_dir: str = "${hydra:runtime.cwd}/data/"
    device: int = 1


@dataclass
class ValidCheckExperimentConfig(BaseExperimentConfig):
    logging_dir: str = "${hydra:run.dir}/"
    path_to_data_dir: str = "${hydra:runtime.cwd}/data/"
    device: int = 1


@dataclass
class CleanRoomExperimentConfig(BaseExperimentConfig):
    logging_dir: str = "${hydra:run.dir}/"
    path_to_data_dir: str = "${hydra:runtime.cwd}/data/"
    device: int = 1


@dataclass
class SorterPlannerExperimentConfig(BaseExperimentConfig):
    logging_dir: str = "${hydra:run.dir}/"
    path_to_data_dir: str = "${hydra:runtime.cwd}/data/"
    device: int = 1


@dataclass
class AgentPlannerExperimentConfig(BaseExperimentConfig):
    logging_dir: str = "${hydra:run.dir}/"
    path_to_data_dir: str = "${hydra:runtime.cwd}/data/"
    device: int = 1


# -------------------------------------------------
# -- Metrics
# -------------------------------------------------


@dataclass
class BaseMetricsConfig:
    _target_: str = MISSING


@dataclass
class LLPMetricsConfig(BaseMetricsConfig):
    _target_: str = "llm_sorter.metrics.LLPMetrics"


@dataclass
class AlfredTaskTypeMetricsConfig(BaseMetricsConfig):
    _target_: str = "llm_sorter.metrics.AlfredTaskTypeMetrics"


@dataclass
class HLPMetricsConfig(BaseMetricsConfig):
    _target_: str = "llm_sorter.metrics.HLPMetrics"


@dataclass
class ValidCheckMetricsConfig(BaseMetricsConfig):
    _target_: str = "llm_sorter.metrics.ValidCheckMetrics"


@dataclass
class CleanRoomMetricsConfig(BaseMetricsConfig):
    _target_: str = "llm_sorter.metrics.CleanRoomMetrics"


@dataclass
class SpecMetricsConfig(BaseMetricsConfig):
    _target_: str = "llm_sorter.metrics.SpecMetrics"


# -------------------------------------------------
# -- Processor
# -------------------------------------------------


@dataclass
class BaseProcessorConfig:
    _target_: str = MISSING
    name: str = MISSING


@dataclass
class LLPProcessorConfig(BaseProcessorConfig):
    _target_: str = "llm_sorter.processors.LLPProcessor"
    name: str = "llp processor"
    run_name: str = "${model.name} ${gen_method.name} ${dataset.dataset_filename}"
    path_to_prompt_dir: Optional[str] = "${hydra:runtime.cwd}/prompts/llp"
    prompt_filename: Optional[str] = "vicuna_prompt.txt"
    load_prompt_from_file: bool = True


@dataclass
class HLPProcessorConfig(BaseProcessorConfig):
    _target_: str = "llm_sorter.processors.HLPProcessor"
    name: str = "hlp processor"
    run_name: str = "${model.name} ${gen_method.name} ${dataset.dataset_filename}"
    path_to_prompt_dir: Optional[str] = "${hydra:runtime.cwd}/prompts/hlp"
    prompt_filename: Optional[str] = "hlp_prompt.txt"
    load_prompt_from_file: bool = True
    n_examples: int = 4


@dataclass
class SpecProcessorConfig(BaseProcessorConfig):
    _target_: str = "llm_sorter.processors.SpecProcessor"
    name: str = "spec processor"
    run_name: str = "${model.name} ${gen_method.name} ${dataset.dataset_filename}"
    path_to_prompt_dir: Optional[str] = "${hydra:runtime.cwd}/prompts/spec"
    # prompt_filename: Optional[str] = "vicuna_prompt.txt"
    prompt_filename: Optional[str] = "spec_prompt.txt"
    load_prompt_from_file: bool = True
    n_examples: int = 10


@dataclass
class AlfredProcessorConfig(BaseProcessorConfig):
    _target_: str = "llm_sorter.processors.LLPProcessor"
    name: str = "llp processor"
    run_name: str = "${model.name} ${gen_method.name} ${dataset.dataset_filename}"
    path_to_prompt_dir: Optional[str] = "${hydra:runtime.cwd}/prompts/alfred"
    prompt_filename: Optional[str] = "vicuna_prompt.txt"
    load_prompt_from_file: bool = False


@dataclass
class AlfredTaskTypeProcessorConfig(BaseProcessorConfig):
    _target_: str = "llm_sorter.processors.AlfredTaskCheckerProcessor"
    name: str = "type checker processor"
    run_name: str = "${model.name} ${gen_method.name} ${dataset.dataset_filename}"
    path_to_prompt_dir: Optional[str] = "${hydra:runtime.cwd}/prompts/alfred"
    prompt_filename: Optional[str] = "vicuna_prompt.txt"
    load_prompt_from_file: bool = False


@dataclass
class ValidCheckProcessorConfig(BaseProcessorConfig):
    _target_: str = "llm_sorter.processors.ValidCheckProcessor"
    name: str = "valid check processor"
    run_name: str = "${model.name} ${gen_method.name} ${dataset.dataset_filename}"
    path_to_prompt_dir: Optional[str] = "${hydra:runtime.cwd}/prompts/valid_check"
    prompt_filename: Optional[str] = "valid_check_good copy.txt"
    load_prompt_from_file: bool = True
    n_examples: int = 10


@dataclass
class CleanRoomProcessorConfig(BaseProcessorConfig):
    _target_: str = "llm_sorter.processors.CleanRoomProcessor"
    name: str = "clean room processor"
    run_name: str = "${model.name} ${gen_method.name} ${dataset.dataset_filename}"
    path_to_prompt_dir: Optional[str] = "${hydra:runtime.cwd}/prompts/clean_room"
    prompt_filename: Optional[str] = "clean_room.txt"
    load_prompt_from_file: bool = True
    n_examples: int = 2


# -------------------------------------------------
# -- Plan generation
# -------------------------------------------------


@dataclass
class BasePlanGenConfig:
    _target_: str = MISSING
    name: str = MISSING


@dataclass
class FullPlanGenerationConfig(BasePlanGenConfig):
    _target_: str = "llm_sorter.gen_methods.FullPlanGeneration"
    name: str = "full generation"


# -------------------------------------------------
# -- Test
# -------------------------------------------------


@dataclass
class TestLLPConfig:
    logger: BaseLoggerConfig = field(default_factory=WandbLoggerConfig)
    experiment: BaseExperimentConfig = field(default_factory=LLPExperimentConfig)
    model: BaseModelConfig = field(default_factory=APIModelConfig)
    metrics: BaseMetricsConfig = field(default_factory=LLPMetricsConfig)
    dataset: BaseDatasetConfig = field(default_factory=LLPTestDatasetConfig)
    processor: BaseProcessorConfig = field(default_factory=LLPProcessorConfig)
    gen_method: BasePlanGenConfig = field(default_factory=FullPlanGenerationConfig)


@dataclass
class TestHLPConfig:
    logger: BaseLoggerConfig = field(default_factory=WandbLoggerConfig)
    experiment: BaseExperimentConfig = field(default_factory=HLPExperimentConfig)
    model: BaseModelConfig = field(default_factory=APIModelConfig)
    metrics: BaseMetricsConfig = field(default_factory=HLPMetricsConfig)
    dataset: BaseDatasetConfig = field(default_factory=HLPDatasetConfig)
    processor: BaseProcessorConfig = field(default_factory=HLPProcessorConfig)
    gen_method: BasePlanGenConfig = field(default_factory=FullPlanGenerationConfig)


@dataclass
class TestSpecConfig:
    logger: BaseLoggerConfig = field(default_factory=WandbLoggerConfig)
    experiment: BaseExperimentConfig = field(default_factory=SpecExperimentConfig)
    model: BaseModelConfig = field(default_factory=APIModelConfig)
    metrics: BaseMetricsConfig = field(default_factory=SpecMetricsConfig)
    dataset: BaseDatasetConfig = field(default_factory=HLPDatasetConfig)
    processor: BaseProcessorConfig = field(default_factory=SpecProcessorConfig)
    gen_method: BasePlanGenConfig = field(default_factory=FullPlanGenerationConfig)


@dataclass
class TestValidCheckConfig:
    logger: BaseLoggerConfig = field(default_factory=WandbLoggerConfig)
    experiment: BaseExperimentConfig = field(default_factory=ValidCheckExperimentConfig)
    model: BaseModelConfig = field(default_factory=APIModelConfig)
    metrics: BaseMetricsConfig = field(default_factory=ValidCheckMetricsConfig)
    dataset: BaseDatasetConfig = field(default_factory=ValidCheckDatasetConfig)
    processor: BaseProcessorConfig = field(default_factory=ValidCheckProcessorConfig)
    gen_method: BasePlanGenConfig = field(default_factory=FullPlanGenerationConfig)


@dataclass
class TestCleanRoomConfig:
    logger: BaseLoggerConfig = field(default_factory=WandbLoggerConfig)
    experiment: BaseExperimentConfig = field(default_factory=CleanRoomExperimentConfig)
    model: BaseModelConfig = field(default_factory=APIModelConfig)
    metrics: BaseMetricsConfig = field(default_factory=CleanRoomMetricsConfig)
    dataset: BaseDatasetConfig = field(default_factory=CleanRoomDatasetConfig)
    processor: BaseProcessorConfig = field(default_factory=CleanRoomProcessorConfig)
    gen_method: BasePlanGenConfig = field(default_factory=FullPlanGenerationConfig)


@dataclass
class TestAlfredConfig:
    logger: BaseLoggerConfig = field(default_factory=WandbLoggerConfig)
    experiment: BaseExperimentConfig = field(default_factory=AlfredExperimentConfig)
    model: BaseModelConfig = field(default_factory=APIModelConfig)
    metrics: BaseMetricsConfig = field(default_factory=LLPMetricsConfig)
    dataset: BaseDatasetConfig = field(default_factory=AlfredDatasetConfig)
    prompt_dataset: BaseDatasetConfig = field(default_factory=AlfredPromptDatasetConfig)
    processor: BaseProcessorConfig = field(default_factory=AlfredProcessorConfig)
    gen_method: BasePlanGenConfig = field(default_factory=FullPlanGenerationConfig)


@dataclass
class TestAlfredTaskTypeConfig:
    logger: BaseLoggerConfig = field(default_factory=WandbLoggerConfig)
    experiment: BaseExperimentConfig = field(default_factory=AlfredExperimentConfig)
    model: BaseModelConfig = field(default_factory=APIModelConfig)
    metrics: BaseMetricsConfig = field(default_factory=AlfredTaskTypeMetricsConfig)  # Fix
    dataset: BaseDatasetConfig = field(default_factory=AlfredDatasetConfig)
    processor: BaseProcessorConfig = field(default_factory=AlfredTaskTypeProcessorConfig)
    gen_method: BasePlanGenConfig = field(default_factory=FullPlanGenerationConfig)


@dataclass
class SorterPlannerConfig:
    logger: BaseLoggerConfig = field(default_factory=WandbLoggerConfig)
    model: BaseModelConfig = field(default_factory=APIModelConfig)
    experiment: BaseExperimentConfig = field(
        default_factory=SorterPlannerExperimentConfig
    )
    gen_method: BasePlanGenConfig = field(default_factory=FullPlanGenerationConfig)

    hlp_processor: BaseProcessorConfig = field(default_factory=HLPProcessorConfig)
    llp_processor: BaseProcessorConfig = field(default_factory=LLPProcessorConfig)
    vc_processor: BaseProcessorConfig = field(default_factory=ValidCheckProcessorConfig)
    spec_processor: BaseProcessorConfig = field(default_factory=SpecProcessorConfig)
    clean_room_processor: BaseProcessorConfig = field(
        default_factory=CleanRoomProcessorConfig
    )

    dataset: BaseDatasetConfig = field(default_factory=SorterDatasetConfig)


@dataclass
class AgentPlannerConfig:
    logger: BaseLoggerConfig = field(default_factory=WandbLoggerConfig)
    model: BaseModelConfig = field(default_factory=APIModelConfig)
    experiment: BaseExperimentConfig = field(
        default_factory=SorterPlannerExperimentConfig
    )
    gen_method: BasePlanGenConfig = field(default_factory=FullPlanGenerationConfig)

    hlp_processor: BaseProcessorConfig = field(default_factory=HLPProcessorConfig)
    llp_processor: BaseProcessorConfig = field(default_factory=LLPProcessorConfig)
    vc_processor: BaseProcessorConfig = field(default_factory=ValidCheckProcessorConfig)

    dataset: BaseDatasetConfig = field(default_factory=SorterDatasetConfig)


# -------------------------------------------------
# -- Inference
# -------------------------------------------------


@dataclass
class InferenceConfig:
    logger: BaseLoggerConfig = field(default_factory=WandbLoggerConfig)
    experiment: BaseExperimentConfig = field(default_factory=BaseExperimentConfig)
