#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import actionlib

from communication_msgs.msg import OpenSeeDSetterAction, OpenSeeDSetterGoal
from communication_msgs.msg import Task, TaskArray


# Функция для обработки сигнала Обратной связи (Action Feadback)
def action_feedback(fb):
    print(fb)


# Функция клиента
def action_client():
    # Создаем клиента для работы с Действиями с именем text_query_generation
    client = actionlib.SimpleActionClient('text_query_generation', OpenSeeDSetterAction)

    # Ожидание подключение к серверу
    client.wait_for_server()

    # Создание задачи на таску

    goal = OpenSeeDSetterGoal()
    #msg.tasks = [Task(type="PICK_UP", object="carrot", location="table"), Task(type="PUT", object="carrot2", location="table2")]
    #msg.tasks = [Task(type="PICK_UP", object="carrot", location="table")]
    #msg.tasks = [Task(type="MOVE_TO", object="unspecified", location="box")]MOVE_TO_RELATIVE
    goal_tasks = TaskArray()
    #goal_tasks.tasks = [Task(type="RECOGNIZE", object="toy cat", location="unspecified"), Task(type="RECOGNIZE", object="objects", location="unspecified")]
    goal_tasks.tasks = [Task(type="RECOGNIZE", object="toy cat", location="unspecified")]
    goal.tasks = goal_tasks

    # Отправка задачи на сервер и определение метода обработки для Обратной связи (Action Feadback)
    client.send_goal(goal, feedback_cb=action_feedback)

    # Ожидание результата (Action Result) работы сервера
    client.wait_for_result()

    # Возвращаем полученный от сервера результат
    return client.get_result()


if __name__ == '__main__':
    try:
        # Инициализация ноды и запуск SimpleActionClient
        rospy.init_node('text_query_generation_client')
        result = action_client()
        print("Result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)