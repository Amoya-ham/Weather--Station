#!/usr/bin/python3

##############################################################################################################
#                                CLASSES CONTAINING ALL THE APP FUNCTIONS                                    #
##############################################################################################################

class DB:

    def __init__(self, Config):
        from math import floor
        from os import getcwd
        from os.path import join
        from json import loads, dumps, dump
        from datetime import timedelta, datetime, timezone 
        from pymongo import MongoClient, errors, ReturnDocument
        from urllib import parse
        from urllib.request import urlopen 
        from bson.objectid import ObjectId  
       
        self.Config          = Config
        self.getcwd          = getcwd
        self.join            = join 
        self.floor           = floor 
        self.loads           = loads
        self.dumps           = dumps
        self.dump            = dump  
        self.datetime        = datetime
        self.ObjectId        = ObjectId 
        self.server          = Config.DB_SERVER
        self.port            = Config.DB_PORT
        self.username        = parse.quote_plus(Config.DB_USERNAME)
        self.password        = parse.quote_plus(Config.DB_PASSWORD)
        self.remoteMongo     = MongoClient
        self.ReturnDocument  = ReturnDocument
        self.PyMongoError    = errors.PyMongoError
        self.BulkWriteError  = errors.BulkWriteError  
        self.tls             = False  # MUST SET TO TRUE IN PRODUCTION

    def __del__(self):
        # Delete class instance to free resources
        pass


   # 1. FUNCTION TO INSERT DATA INTO Weather_Station COLLECTION
    def addUpdate(self,data):
        '''ADD A NEW STORAGE LOCATION TO COLLECTION'''
        try:
            remotedb 	= self.remoteMongo('mongodb://%s:%s@%s:%s' % (self.username, self.password,self.server,self.port), tls=self.tls)
            result      = remotedb.ELET2415.Weather_Sat.insert_one(data)
        except Exception as e:
            msg = str(e)
            if "duplicate" not in msg:
                print("addUpdate error ",msg)
            return False
        else:                  
            return True

    # 2. FUNCTION TO RETRIEVE DOCUMENTS FROM RADAR COLLECTION BETWEEN DATE RANGE
    def getAllInRange(self,start, end):
        '''RETURNS A LIST OF OBJECTS. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        try:
            start = int(start)
            end = int(end)
            remotedb 	= self.remoteMongo('mongodb://%s:%s@%s:%s' % (self.username, self.password,self.server,self.port), tls=self.tls)
            #result      = list(remotedb.ELET2415.climo.find({'timestamp': {'$gte': start, '$lte': end}}, {'_id': 0}).sort({'timestamp': 1}))
            result      = list(remotedb.ELET2415.Weather_Sat.find({'timestamp': {'$gte': start, '$lte': end}},{'_id': 0}).sort('timestamp', 1))
        except Exception as e:
            msg = str(e)
            print("getAllInRange error ",msg)            
        else:                  
            return result
    

    # 3. FUNCTION TO COMPUTE AVERAGE 'reserve' VALUE
    def humidityMMAR(self,start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR HUMIDITY. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        try:
            start = int(start)
            end = int(end)
            remotedb 	= self.remoteMongo('mongodb://%s:%s@%s:%s' % (self.username, self.password,self.server,self.port), tls=self.tls)
            result = list(remotedb.ELET2415.Weather_Sat.aggregate([
            {'$match': {'timestamp': {'$gte': start, '$lte': end}}},
            {'$group': {'_id': 0, 'humidity': {'$push': '$humidity'}}},
            {'$project': {
                'max': {'$max': '$humidity'},
                'min': {'$min': '$humidity'},
                'avg': {'$avg': '$humidity'},
                'range': {'$subtract': [{'$max': '$humidity'}, {'$min': '$humidity'}]}
            }}
        ]))
        except Exception as e:
            msg = str(e)
            print("humidityMMAR error ",msg)            
        else:                  
            return result

    # 4. FUNCTION TO INSERT/UPDATE PASSCODE IN THE 'code' COLLECTION
    def farenheitMMAR(self,start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR HUMIDITY. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        try:
            start = int(start)
            end = int(end)
            remotedb 	= self.remoteMongo('mongodb://%s:%s@%s:%s' % (self.username, self.password,self.server,self.port), tls=self.tls)
            result = list(remotedb.ELET2415.Weather_Sat.aggregate([
            {'$match': {'timestamp': {'$gte': start, '$lte': end}}},
            {'$group': {'_id': 0, 'fahrTemperature': {'$push': '$fahrTemperature'}}},
            {'$project': {
                'max': {'$max': '$fahrTemperature'},
                'min': {'$min': '$fahrTemperature'},
                'avg': {'$avg': '$fahrTemperature'},
                'range': {'$subtract': [{'$max': '$fahrTemperature'}, {'$min': '$fahrTemperature'}]}
            }}
        ]))
        except Exception as e:
            msg = str(e)
            print("farenheitMMAR error ",msg)            
        else:                  
            return result
        

    # 5. FUNCTION TO COUNT PASSCODES IN 'code' COLLECTION
    def temperatureMMAR(self,start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR TEMPERATURE. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        try:
            start = int(start)
            end = int(end)
            remotedb 	= self.remoteMongo('mongodb://%s:%s@%s:%s' % (self.username, self.password,self.server,self.port), tls=self.tls)
            #print(type(start))
            #print(type(end))
            #print(start)
            #print(end)
            result = list(remotedb.ELET2415.Weather_Sat.aggregate([
            {'$match': {'timestamp': {'$gte': start, '$lte': end}}},
            {'$group': {'_id': 'celsTemperature', 'celsTemperature': {'$push': '$celsTemperature'}}},
            {'$project': {
                'max': {'$max': '$celsTemperature'},
                'min': {'$min': '$celsTemperature'},
                'avg': {'$avg': '$celsTemperature'},
                'range': {'$subtract': [{'$max': '$celsTemperature'}, {'$min': '$celsTemperature'}]}
            }}
        ]))
            
            print(result)
        except Exception as e:
            msg = str(e)
            print("temperatureMMAR error ",msg)            
        else:                 
            return result
        
def HeatIndexMMAR(self,start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR HUMIDITY. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        try:
            start = int(start)
            end = int(end)
            remotedb 	= self.remoteMongo('mongodb://%s:%s@%s:%s' % (self.username, self.password,self.server,self.port), tls=self.tls)
            result = list(remotedb.ELET2415.Weather_Sat.aggregate([
            {'$match': {'timestamp': {'$gte': start, '$lte': end}}},
            {'$group': {'_id': 0, 'heatindex': {'$push': '$heatindex'}}},
            {'$project': {
                'max': {'$max': '$heatindex'},
                'min': {'$min': '$heatindex'},
                'avg': {'$avg': '$heatindex'},
                'range': {'$subtract': [{'$max': '$heatindex'}, {'$min': '$heatindex'}]}
            }}
        ]))
            
        except Exception as e:
            msg = str(e)
            print("HeatIndexMMAR error ",msg)            
        else:                  
            return result
        
def pressureMMAR(self,start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR HUMIDITY. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        try:
            start = int(start)
            end = int(end)
            remotedb 	= self.remoteMongo('mongodb://%s:%s@%s:%s' % (self.username, self.password,self.server,self.port), tls=self.tls)
            result = list(remotedb.ELET2415.Weather_Sat.aggregate([
            {'$match': {'timestamp': {'$gte': start, '$lte': end}}},
            {'$group': {'_id': 0, 'pressure': {'$push': '$pressure'}}},
            {'$project': {
                'max': {'$max': '$pressure'},
                'min': {'$min': '$pressure'},
                'avg': {'$avg': '$pressure'},
                'range': {'$subtract': [{'$max': '$pressure'}, {'$min': '$pressure'}]}
            }}
        ]))
            
        except Exception as e:
            msg = str(e)
            print("pressureMMAR error ",msg)            
        else:                  
            return result
        
def altitudeMMAR(self,start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR HUMIDITY. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        try:
            start = int(start)
            end = int(end)
            remotedb 	= self.remoteMongo('mongodb://%s:%s@%s:%s' % (self.username, self.password,self.server,self.port), tls=self.tls)
            result = list(remotedb.ELET2415.Weather_Sat.aggregate([
            {'$match': {'timestamp': {'$gte': start, '$lte': end}}},
            {'$group': {'_id': 0, 'altitude': {'$push': '$altitude'}}},
            {'$project': {
                'max': {'$max': '$altitude'},
                'min': {'$min': '$altitude'},
                'avg': {'$avg': '$altitude'},
                'range': {'$subtract': [{'$max': '$altitude'}, {'$min': '$altitude'}]}
            }}
        ]))
            
        except Exception as e:
            msg = str(e)
            print("altitudeMMAR error ",msg)            
        else:                  
            return result
        
def soilmoistureMMAR(self,start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR HUMIDITY. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        try:
            start = int(start)
            end = int(end)
            remotedb 	= self.remoteMongo('mongodb://%s:%s@%s:%s' % (self.username, self.password,self.server,self.port), tls=self.tls)
            result = list(remotedb.ELET2415.Weather_Sat.aggregate([
            {'$match': {'timestamp': {'$gte': start, '$lte': end}}},
            {'$group': {'_id': 0, 'soilMoisture': {'$push': '$soilMoisture'}}},
            {'$project': {
                'max': {'$max': '$soilMoisture'},
                'min': {'$min': '$soilMoisture'},
                'avg': {'$avg': '$soilMoisture'},
                'range': {'$subtract': [{'$max': '$soilMoisture'}, {'$min': '$soilMoisture'}]}
            }}
        ]))
            
        except Exception as e:
            msg = str(e)
            print("soilmoistureMMAR error ",msg)            
        else:                  
            return result
    
def frequencyDistro(self,variable,start, end):
        '''RETURNS THE FREQUENCY DISTROBUTION FOR A SPECIFIED VARIABLE WITHIN THE START AND END DATE RANGE'''
        try:
            start=int(start)
            end=int(end)
            remotedb 	= self.remoteMongo('mongodb://%s:%s@%s:%s' % (self.username, self.password,self.server,self.port), tls=self.tls)
            result = list(remotedb.ELET2415.Weather_Sat.aggregate([
            {'$match': {'timestamp': {'$gte': start, '$lte': end}}},
            {'$bucket': {
                'groupBy': f"${variable}",
                'boundaries': list(range(101)),
                'default': 'outliers',
                'output': {'count': {'$sum': 1}}
            }}
        ]))
        
        except Exception as e:
            msg = str(e)
            print("frequencyDistro error ",msg)            
        else:                  
            return result

        

def main():
    from config import Config
    from time import time

    db = DB(Config)

    start = time()
    # You can add test cases here to call class methods
    end = time()

    print(f"Completed in: {end - start} seconds")
    
if __name__ == '__main__':
    main()
