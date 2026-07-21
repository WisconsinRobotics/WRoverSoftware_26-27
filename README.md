# WRoverSoftware_26-27
Official Wisconsin Robotics software repository for the 2027 University Rover Challenge.

## Requirements

- All dependencies should be handled by the docker container. To see how to run this repo in the docker container, see the [WRoverSoftware_Docker repo](https://github.com/WisconsinRobotics/WRoverSoftware_Docker).

- If you need to install python packages, you must add it to `requirements.txt` in the docker repo. Then you must rebuild the docker container and open a PR for the changes you made. This is also specified in the docker repo.

## Setup

- Make sure that you are running the docker container.

- **Inside** the container, clone the repo into the workspace directory:

  ```bash
  git clone git@github.com:WisconsinRobotics/WRoverSoftware_26-27.git
  ```

- Initialize pre-commit hooks:

  ```bash
  pre-commit install
  ```

- To run pre-commit checks manually:

  ```bash
  pre-commit run -a
  ```

  > We use pre-commit hooks to format code and check for simple mistakes. They automatically run before every commit.
