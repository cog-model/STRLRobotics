from dataclasses import dataclass, field
from enum import auto, Enum
from typing import List
import json


class Items(Enum):
    tomato = auto
    cucumber = auto()
    toy_cat = auto()
    cat = auto()
    stuffed_cat = auto()
    orange_cat = auto()
    cube = auto()
    toy_box = auto()
    red_box = auto()
    orange_box = auto()
    apple = auto()
    pear = auto()
    banana = auto()
    socks = auto()
    t_shirt = auto()
    pepper = auto()
    bottle = auto()
    can = auto()
    green_sweet_pepper = auto()


@dataclass
class Subtask:
    text: str = ""

    def to_json(self) -> List:
        return self.text


@dataclass
class Task:
    goal_eng: str = ""
    request: str = ""
    explanation: str = ""
    items: List[Items] = field(default_factory=list)
    subtasks: List[Subtask] = field(default_factory=list)
    valid_plan: bool = True

    def to_json(self, id: int = 0):
        out = {}
        out["task_type"] = "test"
        out["plan_id"] = id
        out["goal_eng"] = self.goal_eng
        out["items"] = [item.name.lower().replace("_", " ") for item in self.items]
        out["request"] = self.request
        out["valid_plan"] = self.valid_plan

        plan = []
        for subtask in self.subtasks:
            plan.append(subtask.to_json())
        out["plan"] = plan

        return out


tests: List[Task] = [
    # Unvalid plans
    Task(goal_eng="Build a pyramid of stuff", valid_plan=False),
    Task(goal_eng="Take the kids to school", valid_plan=False),
    Task(goal_eng="Make me a coffee", valid_plan=False),
    Task(goal_eng="Turn on the music", valid_plan=False),
    Task(goal_eng="Turn on the vacuum cleaner", valid_plan=False),
    Task(goal_eng="Write a handwritten letter", valid_plan=False),
    Task(
        goal_eng="Give a detailed explanation of a complex philosophical concept",
        valid_plan=False,
    ),
    Task(goal_eng="Create a piece of abstract art", valid_plan=False),
    Task(goal_eng="Comfort someone going through emotional distress", valid_plan=False),
    Task(goal_eng="Play a musical instrument with expressiveness", valid_plan=False),
    Task(goal_eng="Learn a new skill by watching a video tutorial", valid_plan=False),
    Task(goal_eng="Engage in a philosophical debate", valid_plan=False),
    Task(goal_eng="Understand and appropriately respond to sarcasm", valid_plan=False),
    Task(goal_eng="Water the plants", valid_plan=False),
    Task(goal_eng="Fold the laundry and organize it in the closet", valid_plan=False),
    Task(goal_eng="Cook a three-course meal following a recipe", valid_plan=False),
    Task(goal_eng="Mow the lawn and trim the edges", valid_plan=False),
    Task(goal_eng="Paint a room in two different colors", valid_plan=False),
    Task(
        goal_eng="Assemble a piece of furniture using the provided instructions",
        valid_plan=False,
    ),
    Task(goal_eng="Solve a Sudoku puzzle", valid_plan=False),
    Task(goal_eng="Write a Python program to calculate prime numbers", valid_plan=False),
    Task(goal_eng="Read a book and write a summary of each chapter", valid_plan=False),
    Task(goal_eng="Learn a new language using a language learning app", valid_plan=False),
    Task(goal_eng="Plant flowers in the garden", valid_plan=False),
    Task(goal_eng="Create a budget and track expenses for the month", valid_plan=False),
    Task(
        goal_eng="Exercise for 30 minutes following a workout routine", valid_plan=False
    ),
    Task(goal_eng="Write a thank-you letter to a friend", valid_plan=False),
    Task(goal_eng="Play a musical instrument following sheet music", valid_plan=False),
    Task(goal_eng="Wash the dishes", valid_plan=False),
    Task(goal_eng="Sort the books on the shelf by genre", valid_plan=False),  # ###
    Task(
        goal_eng="Organize a closet by color and clothing type", valid_plan=False
    ),  # ###
    Task(goal_eng="Set up a chessboard for a game", valid_plan=False),  # ###
    Task(
        goal_eng="Write a Python script to automate a repetitive task", valid_plan=False
    ),
    Task(goal_eng="Create a weekly meal plan and grocery list", valid_plan=False),
    Task(goal_eng="Assemble a jigsaw puzzle", valid_plan=False),
    Task(goal_eng="Teach a child to tie their shoelaces", valid_plan=False),
    Task(goal_eng="Perform a magic trick with playing cards", valid_plan=False),
    Task(goal_eng="Design a logo for a new business", valid_plan=False),
    Task(goal_eng="Plan a day trip to a nearby city", valid_plan=False),
    Task(
        goal_eng="Compile a list of recommended books for a specific genre",
        valid_plan=False,
    ),
    Task(goal_eng="Write a blog post on a topic of personal interest", valid_plan=False),
    Task(goal_eng="Create a budget for a home improvement project", valid_plan=False),
    Task(goal_eng="Plan and execute a picnic in the park", valid_plan=False),
    Task(
        goal_eng="Learn to play a simple tune on a musical instrument", valid_plan=False
    ),
    Task(goal_eng="Solve a crossword puzzle", valid_plan=False),
    Task(goal_eng="Plant herbs in a kitchen garden", valid_plan=False),
    Task(goal_eng="Paint a canvas with acrylics", valid_plan=False),
    Task(goal_eng="Record a short video tutorial for a basic skill", valid_plan=False),
    Task(goal_eng="Build a sandcastle", valid_plan=False),
    Task(goal_eng="Teach a dog to do a backflip", valid_plan=False),
    Task(goal_eng="Write a poem about the meaning of life", valid_plan=False),
    Task(goal_eng="Teach a robot to feel emotions", valid_plan=False),
    Task(goal_eng="Give a TED talk on quantum physics", valid_plan=False),
    Task(goal_eng="Perform a stand-up comedy routine", valid_plan=False),
    Task(goal_eng="Teach a computer to appreciate art", valid_plan=False),
    Task(goal_eng="Fly to the moon and back", valid_plan=False),
    Task(
        goal_eng="Translate a sentence into a language that doesn't exist",
        valid_plan=False,
    ),
    Task(goal_eng="Invent a perpetual motion machine", valid_plan=False),
    Task(goal_eng="Build a time machine", valid_plan=False),
    Task(goal_eng="Predict the future with 100% accuracy", valid_plan=False),
    Task(goal_eng="Create a device to read minds", valid_plan=False),
    Task(goal_eng="Bake a cake using only thoughts", valid_plan=False),
    Task(goal_eng="Speak fluent dolphin", valid_plan=False),
    Task(goal_eng="Convince a rock to move", valid_plan=False),
    Task(goal_eng="Write a symphony for whales", valid_plan=False),
    Task(goal_eng="Travel to a parallel universe", valid_plan=False),
    Task(goal_eng="Make a snowman in the desert", valid_plan=False),
    Task(goal_eng="Teach a robot to appreciate abstract art", valid_plan=False),
    Task(goal_eng="Wash the dishes", valid_plan=False),
    Task(goal_eng="Sort the books on the shelf by genre", valid_plan=False),
    Task(goal_eng="Organize a closet by color and clothing type", valid_plan=False),
    Task(goal_eng="Set up a chessboard for a game", valid_plan=False),
    Task(
        goal_eng="Write a Python script to automate a repetitive task", valid_plan=False
    ),
    Task(goal_eng="Create a weekly meal plan and grocery list", valid_plan=False),
    Task(goal_eng="Assemble a jigsaw puzzle", valid_plan=False),
    Task(goal_eng="Teach a child to tie their shoelaces", valid_plan=False),
    Task(goal_eng="Perform a magic trick with playing cards", valid_plan=False),
    Task(goal_eng="Design a logo for a new business", valid_plan=False),
    Task(goal_eng="Plan a day trip to a nearby city", valid_plan=False),
    Task(goal_eng="Compile a list of recommended books for a  genre", valid_plan=False),
    Task(goal_eng="Write a blog post on a topic of personal interest", valid_plan=False),
    Task(goal_eng="Create a budget for a home improvement project", valid_plan=False),
    Task(goal_eng="Plan and execute a picnic in the park", valid_plan=False),
    Task(
        goal_eng="Learn to play a simple tune on a musical instrument", valid_plan=False
    ),
    Task(goal_eng="Solve a crossword puzzle", valid_plan=False),
    Task(goal_eng="Plant herbs in a kitchen garden", valid_plan=False),
    Task(goal_eng="Paint a canvas with acrylics", valid_plan=False),
    Task(goal_eng="Record a short video tutorial for a basic skill", valid_plan=False),
    Task(goal_eng="Sweep the kitchen floor", valid_plan=False),
    Task(goal_eng="Organize the bookshelf alphabetically", valid_plan=False),
    Task(goal_eng="Prepare a sandwich with ham and cheese", valid_plan=False),
    Task(goal_eng="Water the garden", valid_plan=False),
    Task(goal_eng="Vacuum the living room", valid_plan=False),
    Task(goal_eng="Sort the recycling into bins", valid_plan=False),
    Task(goal_eng="Wash and dry the dishes", valid_plan=False),
    Task(goal_eng="Make a bed with clean sheets", valid_plan=False),
    Task(goal_eng="Set the table for dinner", valid_plan=False),
    Task(goal_eng="Feed the pet fish", valid_plan=False),
    Task(goal_eng="Write a Python script to automate a task", valid_plan=False),
    Task(goal_eng="Fold and put away the laundry", valid_plan=False),
    Task(goal_eng="Clean the car interior", valid_plan=False),
    Task(goal_eng="Rearrange the furniture in the living room", valid_plan=False),
    Task(goal_eng="Paint a canvas using acrylics", valid_plan=False),
    Task(goal_eng="Create a shopping list for the week", valid_plan=False),
    Task(goal_eng="Fix a leaky faucet", valid_plan=False),
    Task(goal_eng="Change the lightbulbs in the hallway", valid_plan=False),
    Task(goal_eng="Buy groceries for the week", valid_plan=False),
    Task(goal_eng="Set up a home security system", valid_plan=False),
]
# You have a home robot equipped with three actions: move_to(object, location), pick(object, location), and put(object, location). The robot is capable of navigating within a defined area, picking up objects at specific locations, and placing them at other locations.
dataset = []
for i, task in enumerate(tests):
    dataset.append(task.to_json(i))
    print(task.goal_eng)

with open("./data/hlp/unvalid_check.json", "w", encoding="utf8") as f:
    json.dump(dataset, f, indent=4, ensure_ascii=False)
