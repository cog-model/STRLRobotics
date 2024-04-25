# OpenSeeD model

## Description
Source code of [OpenSeeD model](https://github.com/IDEA-Research/OpenSeeD) necessary for basic inference.

## Installation
- Create a conda environment
```bash
conda create --name openseed python=3.8 -y
conda activate openseed
```
- Install packages and other dependencies.
```bash
pip3 install torch==1.13.1 torchvision==0.14.1 --extra-index-url https://download.pytorch.org/whl/cu113
python -m pip install 'git+https://github.com/MaureenZOU/detectron2-xyz.git'
pip install git+https://github.com/cocodataset/panopticapi.git
python -m pip install -r requirements.txt
```
- Install [rospkg](https://anaconda.org/conda-forge/rospkg) to use ROS inside the conda environment
```bash
conda install -c conda-forge rospkg
```

## Usage

- Choose config and checkpoint:

| Method | Backbone | Crop Size | Dataset   | mAP(mask)0.5...0.95   | mAP(mask)0.5   | AP(mask) (toy block) | config | Checkpoint |
|   :---:| :---:    |  :---:    |  :---:    | :---:                 |           :---:| :---:                | :---:  | :---:      |
| OpenSeeD | Swin-T<sup>&dagger;</sup> | 512&times;512 | Benchamrk Val (v0 objects) | 64.7| 81 | 54.6 |[config](configs/openseed_swint_lang_rosbag.yaml) | [model](https://drive.google.com/file/d/1eRrlvM6y1uWxpVzuPrEgESS_OyTlmEUX/view?usp=drive_link) |

- Change paths to config and checkpoint inside [initialization](https://git.sberrobots.ru/mipt.navigation/object_segmentation/husky_tidy_bot_cv/-/blob/openseed/scripts/openseed_node.py) of the OpenSeeD ROS node.

- Chage path to OpenSeeD source code inside OpenSeeD [wrapper](https://git.sberrobots.ru/mipt.navigation/object_segmentation/husky_tidy_bot_cv/-/blob/openseed/scripts/openseed_model.py)
 
## Support
Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc.

## Roadmap
If you have ideas for releases in the future, it is a good idea to list them in the README.

## Contributing
State if you are open to contributions and what your requirements are for accepting them.

For people who want to make changes to your project, it's helpful to have some documentation on how to get started. Perhaps there is a script that they should run or some environment variables that they need to set. Make these steps explicit. These instructions could also be useful to your future self.

You can also document commands to lint the code or run tests. These steps help to ensure high code quality and reduce the likelihood that the changes inadvertently break something. Having instructions for running tests is especially helpful if it requires external setup, such as starting a Selenium server for testing in a browser.

## Authors and acknowledgment
[OpenSeeD model](https://github.com/IDEA-Research/OpenSeeD)

## License
For open source projects, say how it is licensed.

## Project status
v0.