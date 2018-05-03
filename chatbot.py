from datetime import datetime
import time

class EventDrivenChatBot:
    
    def __init__(self):
        self.accepted_messages = {}
        
        # 1. ADDED THIS "STATE" VARIABLE
        self.has_been_asked_age = False
        
        self.birth_time = datetime.now()
        
        # "registering" all callbacks
        self.register_callback("hi", 
                               self.respond_to_greeting)
        self.register_callback("bye", 
                               self.respond_to_departure)
        
        # 3. USING handle_age_request TO DISPATCH
        #    RESPONSES TO "age?"
        self.register_callback("age?",
                               self.handle_age_request)
    
    def register_callback(self, message, callback):
        """
        Registers a callback to a message.
        """
        if message not in self.accepted_messages:
            self.accepted_messages[message] = []
        self.accepted_messages[message].append(callback)
        
    def respond_to_greeting(self):
        print("Hello!")
        
    def respond_to_departure(self):
        print("Nice chatting with you!")
    
    # 2. ADD DISPATCH POINT FOR PROCESSING "age?" MESSAGE
    def handle_age_request(self):
        if not self.has_been_asked_age:
            self.has_been_asked_age = True
            self.respond_to_age_request()
        else:
            self.respond_to_age_request_detailed()
            
    def respond_to_age_request(self):
        age = datetime.now() - self.birth_time
        print("I am", age.seconds, "seconds old.")
        
    def respond_to_age_request_detailed(self):
        age = datetime.now() - self.birth_time
        micros = age.microseconds
        print("Technically, I'm", age.seconds, "seconds and", 
              micros, "microseconds old")
        
    def handle_message(self, message):
        if message not in self.accepted_messages:
            print("sorry, I don't understand", message)
        else:
            callbacks = self.accepted_messages[message]
            for callback in callbacks:
                callback()
                
bot = EventDrivenChatBot()
bot.handle_message("hi")
time.sleep(1.3)
bot.handle_message("age?")
print("---No chatbot, let me ask you that again...")
bot.handle_message("age?")