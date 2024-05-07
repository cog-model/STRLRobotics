from typing import List, Optional
import requests
from time import sleep, time
import logging

logging.basicConfig(
    filename="./log.txt",
    filemode='a',
    format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
    datefmt='%H:%M:%S',
    level=logging.INFO
)

logger = logging.getLogger('TEST')
print = logger.info
print("\n\n======================================New Log Session======================================\n\n")

def test_low_level(text):
    url = "http://0.0.0.0:8082/add_llp_task"

    tic = time()
    data = {"goal": text}
    answer = requests.post(url, json=data)
    answer = answer.text
    toc = time()
    print(toc - tic)
    print(answer)


def test_high_level(text, request_answer: Optional[List[str]] = None):
    url = "http://0.0.0.0:8082/add_task"

    tic = time()
    data = {"goal": text}
    answer = requests.post(url, json=data)
    toc = time()
    print(f"Goal request: {data['goal']}, Time: {toc - tic: 0.2f}")
    if answer.text != "null":
        print(answer)
    if request_answer is not None:
        sleep(4)
        tic = time()
        data = {"feedback": request_answer}
        url = "http://0.0.0.0:8082/add_feedback"
        answer = requests.post(url, json=data)
        toc = time()
        print(f"Feedback request. Time: {toc - tic: 0.2f}")
        if answer.text != "null":
            print(answer)

    sleep(2)
    for _ in range(20):
        url = "http://0.0.0.0:8082/get_planner_output"
        # tic = time()
        answer = requests.get(url)
        answer = answer.json()
        if answer is None:
            sleep(2)
        elif answer["data"] == "empty":
            break
        elif answer["type"] == "Done":
            print(answer)
            print("\n")
        else:
            print(answer)
        sleep(2)


def print_tasks():
    url = "http://0.0.0.0:8082/do_planning"

    tic = time()
    answer = requests.post(url)
    answer = answer.text
    toc = time()
    print(toc - tic)
    print(answer)


def reset():
    url = "http://0.0.0.0:8082/reset"

    answer = requests.post(url)
    answer = answer.text


def test_colors():
    print("===================Running color TEST===================")
    
    objects = ["object", "toy", "vegetable", "dish", "cloth", "fruit", "animal"]

    objects_specs = ["green", "red", "yellow", "white", "black"]
    texts = [
        "Put {} from the chair on the floor",
        "Put {} in the green container",
        "Put {} in the white container",
        "Put {} in the black container",
        "Put the {} picked up from the floor in the red container"
    ]
    
    # ==========Test one object==========
    
    for obj in objects:
        for text in texts:
            for spec in objects_specs:
                object_in_prompt = spec + " " + obj
                prompt = text.format(object_in_prompt)
                test_high_level(prompt)
                sleep(2)

def run_test():
    print("===================Running general TEST===================")
    
    objects = ["object", "toy", "vegetable", "dish", "cloth", "fruit", "animal"]
    objects_plural = ["objects", "toys", "vegetables", "dishes", "clothes", "fruits", "animals"]
    
    objects_num = [1, 2, 3, 4]
    objects_specs = ["green", "red", "yellow", "white", "black", "plush"]
    texts = [
        "Put {} in the container",
        "Put {} in the green container",
        "Put {} in the white container",
        "Put {} in the black container",
        "Put {} from the chair on the floor",
        "Put the {} picked up from the floor in the red container",
        "Put a {} from the bedside table in the drawer, then take a plastic red cup off on the armchair and put it in the container"
    ]
    
    # ==========Test one object==========
    
    for obj in objects:
        for text in texts:
            for spec in objects_specs:
                object_in_prompt = spec + " " + obj
                prompt = text.format(object_in_prompt)
                test_high_level(prompt)
                sleep(2)
    
    
    # ==========Test multiple object==========
    
    for obj in objects_plural:
        for obj_num in objects_num:
            for text in texts:
                object_in_prompt = "all the {}".format(obj)
                prompt = text.format(object_in_prompt)
                request_answer = [
                    obj + " " + str(num+1) 
                    for num in range(obj_num)
                ]
                test_high_level(prompt, request_answer)
                sleep(2)
    


if __name__ == "__main__":
    reset()
    # test_high_level("Put all toy cubes in the box", ["toy cube 1", "toy cube 2"])
    #
    # sleep(2)
    # test_high_level("Put toy cat in the box, then put  toy cubes in the box")
    # sleep(2)
    # test_high_level("Put toy cat on the floor")
    # sleep(2)
    # test_high_level("Pour water over the person")
    # sleep(2)
    # test_high_level("Clean room")
    # sleep(2)
    # test_high_level("Clean all room")
    # test_high_level("put the yellow dish in the green container")
    # test_high_level("put the yellow dish in the white container")
    # test_high_level("put the yellow dish in the black container")
    run_test()
    # test_colors()
    reset()


    