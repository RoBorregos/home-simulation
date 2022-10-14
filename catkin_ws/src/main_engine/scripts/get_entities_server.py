#! /usr/bin/env python

import rospy

import actionlib
import rospy
import requests
from std_msgs.msg import String, Bool
import os
import json
import sys
from main_engine.msg import get_entitiesFeedback, get_entitiesAction, get_entitiesResult, entities


class Parser(object):
    JSON_FILENAME = 'possible_actions.json'
    REST_ENDPOINT = "http://localhost:5005/webhooks/rest/webhook"
    PARSE_ENDPOINT = "http://localhost:5005/model/parse"
    START_TALK = None
    DEBUG = True
    conversationStarted = False

    def __init__(self, START_TALK):
        self.START_TALK = START_TALK
        self.say_publisher = rospy.Publisher('robot_text', String, queue_size=10)
        # self.possible_actions = self.loadActions()
        # self.actions_publisher = rospy.Publisher('action/bring_something', bring_something_cmd, queue_size=10)
        # self.input_suscriber = rospy.Subscriber("RawInput", RawInput, self.callback)
        if self.START_TALK:
            self.someone_to_talk_suscriber = rospy.Subscriber('someoneToTalkStatus', Bool, self.someone_to_talk_callback)

    def loadActions(self):
        '''
        Returns a dictionary with the possible actions.
        {
            cmd_id:{ cmd_category, cmd_priority, require_args }
        }
        '''
        directory = os.path.dirname(os.path.realpath(__file__))
        absolute_path = os.path.join(directory, self.JSON_FILENAME)
        dictionary_possible_actions = dict()
        with open(absolute_path, 'r') as actions_file:
            dictionary_possible_actions = json.load(actions_file)
        return dictionary_possible_actions

    def debug(self, text):
        if(self.DEBUG):
            rospy.loginfo(text)

    def say(self, text):
        response = String(text)
        self.say_publisher.publish(response)

    def callRASA(self, command, silence = False):
        intent = ""
        args = []
        if not silence:
            self.say("You just said:" + command)
        data = { 
            "sender": "HOME", 
            "message": command 
        }

        rest_response = requests.post(self.REST_ENDPOINT, json = data)
        parse_response = requests.post(self.PARSE_ENDPOINT, json = { "text": command })

        if(rest_response.status_code == 200 and parse_response.status_code == 200):
            if(len(rest_response.json()) > 0 and len(parse_response.json()) > 0):
                for responseData in rest_response.json():
                    self.debug("BOT SAYS: " + responseData["text"])
                    self.say(responseData["text"])
                  
                nlu_info = parse_response.json()
                if(nlu_info["intent"]["confidence"] >= 0.60):
                    self.debug("Intent: " + str(nlu_info["intent"]))
                    self.debug("Entities: " + str(nlu_info["entities"]))
                    intent = nlu_info["intent"]["name"]
                    args = nlu_info["entities"]
            entities = {}

            for entitie in nlu_info["entities"]:
                entities[entitie["entity"]] = entitie["value"]
            
        return entities
        # else:
        #     self.say("I'm sorry, Could you rephrase?")
        #     self.debug("Cant connect to server")

        return intent, args

class GetEntitiesAction(object):
    # create messages that are (used to publish feedback/result
    _feedback = get_entitiesFeedback()
    _result = get_entitiesResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, get_entitiesAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    
    def resultHandler(self, result):
        hres = get_entitiesResult()
        
        for name,value in result.items():
            setattr(hres.entities, name, value)

        return hres
        
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        rospy.logwarn(goal.instruction)
        parser = Parser(False)
        self._result = self.resultHandler(parser.callRASA(goal.instruction))


        # start executing the action
        r.sleep()

        
          
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('get_entities_actionserver')
    server = GetEntitiesAction(rospy.get_name())
    rospy.spin()

    


