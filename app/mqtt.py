########################################################################################################
#                                                                                                      #
#   MQTT Paho Documentation - https://eclipse.dev/paho/index.php?page=clients/python/docs/index.php    #
#                                                                                                      #
########################################################################################################
import paho.mqtt.client as mqtt
from random import randint
from json import dumps, loads
from time import sleep
# from .config import Config
# from .functions import DB 

class MQTT:

    # ID = f"IOT_B_1000"
    ID = f"IOT_B_{randint(1,1000000)}"

    #  DEFINE ALL TOPICS TO SUBSCRIBE TO
    sub_topics = [("620161390_pub", 0), ("620161390", 0), ("620161390_sub", 0)] #  A list of tuples of (topic, qos). Both topic and qos must be present in the tuple.


    def __init__(self,mongo):
       
        self.randint                = randint
        self.loads                  = loads
        self.dumps                  = dumps
        self.sleep                  = sleep
        self.mongo                  = mongo
        self.client                 = mqtt.Client(client_id= self.ID, clean_session=True, reconnect_on_failure = True)
        self.client.on_connect      = self.on_connect
        self.client.on_message      = self.on_message
        self.client.on_disconnect   = self.on_disconnect
        self.client.on_subscribe    = self.on_subscribe


        # REGISTER CALLBACK FUNCTION FOR EACH TOPIC
        self.client.message_callback_add("620161390", self.update)
        self.client.message_callback_add("620161390_pub", self.toggle)

        # ADD MQTT SERVER AND PORT INFORMATION BELOW
        self.client.connect_async("broker.emqx.io", 1883, 60)
       


    def connack_string(self,rc):# Allows one to map MQTT connection return codes to human-readable message.
        connection = {0: "Connection successful", 1: "Connection refused - incorrect protocol version", 2: "Connection refused - invalid client identifier", 3: "Connection refused - server unavailable", 4: "Connection refused - bad username or password", 5: "Connection refused - not authorised" }
        return connection[rc]

 
    def on_connect(self,client, userdata, flags, rc):# This gets called when an MQTT Client connects succesfullly to the broker. 
        # Called when the broker responds to our connection request.
        print("\n\nMQTT: "+ self.connack_string(rc)," ID: ",client._client_id.decode('utf-8'))
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe(self.sub_topics)     
 
    def on_subscribe(self, client, userdata, mid, granted_qos): # used to indicate when a MQTT broker acknowledges a subcription request then after the client would be sc=ubcribed to a topic.  
        # Called when the broker responds to a subscribe request.   
        print("MQTT: Subscribed to", [topic[0] for topic in self.sub_topics])

    def publish(self,topic,payload):# Responsible for publishing message such as ensuring the messages are sent properly and handles errrors if publishing fails. 
        try :
            info = self.client.publish(topic, payload)
            info.wait_for_publish()
            return info.is_published()
        
        except Exception as e:
            print(f"MQTT: Publish failed {str(e)}")


    def on_message(self,client, userdata, msg):# Handles the incoming messages and unexpected disconnections.
        # The callback for when a PUBLISH message is received from the server.
        try:
            print(msg.topic+" "+str(msg.payload.decode("utf-8")))
        except Exception as e:
            print(f"MQTT: onMessage Error: {str(e)}")

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("MQTT: Unexpected Disconnection.")
   

    # DEFINE CALLBACK FUNCTIONS FOR EACH TOPIC
    def update(self, client, userdata, msg):# processes the incoming MQTT messages exrtract the data then stores in MongoDB Database.
        try:
            topic   = msg.topic
            payload = msg.payload.decode("utf-8")
            # print(payload) # UNCOMMENT WHEN DEBUGGING  
            
            update  = loads(payload) # CONVERT FROM JSON STRING TO JSON OBJECT  
            self.mongo.addUpdate(update) # INSERT INTO DATABASE
            print(update) 

        except Exception as e:
            print(f"MQTT: GDP Error: {str(e)}")

    def toggle(self,client, userdata, msg): # gets messages from the frontend, extracts the payload then converst it into an JASOn OBJECT
        '''Process messages from Frontend'''
        try:
            topic   = msg.topic
            payload = msg.payload.decode("utf-8")
            # print(payload) # UNCOMMENT WHEN DEBUGGING
            update  = loads(payload) # CONVERT FROM JSON STRING TO JSON OBJECT              
            print(update)

        except Exception as e:
            print(f"MQTT: toggle Error - {str(e)}")



     

