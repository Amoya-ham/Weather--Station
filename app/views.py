"""
Flask Documentation:     https://flask.palletsprojects.com/
Jinja2 Documentation:    https://jinja.palletsprojects.com/
Werkzeug Documentation:  https://werkzeug.palletsprojects.com/
This file creates your application.
"""

# from crypt import methods
import site 

from app import app, Config,  mongo, Mqtt
from flask import render_template, request, jsonify, send_file, redirect, make_response, send_from_directory 
from json import dumps, loads 
from werkzeug.utils import secure_filename
from datetime import datetime,timedelta, timezone
from os import getcwd
from os.path import join, exists
from time import time, ctime
from math import floor

from markupsafe import escape



#####################################
#   Routing for your application    #
#####################################


# 1. CREATE ROUTE FOR 'temperature'
@app.route('/api/mmar/temperature/<start>/<end>', methods=['GET']) 
def gettemperatureMMAR(start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR TEMPERATURE. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        start = int(start)
        end = int(end)
            # remotedb 	= self.remoteMongo('mongodb://%s:%s@%s:%s' % (self.username, self.password,self.server,self.port), tls=self.tls)
        print(type(start))
        print(type(end))
        print(start)            
        print(end)
        if request.method == "GET":
            try:
                val= mongo.temperatureMMAR(start, end)
                print(f"val: {val}")
                if val:

                    return jsonify({"status":"found","data":val})
                    print("No data")
            except Exception as e:
                msg = str(e)
                print(f"get_all error: f{str(e)}")

        return jsonify({"status":"not found","data":[]})

# 2. CREATE ROUTE FOR 'humidity'
@app.route('/api/mmar/humidity/<start>/<end>', methods=['GET'])
def gethumidityMMAR(start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR HUMIDITY. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        start= int(start)
        end= int(end)

        if request.method == "GET":
            try:
                val= mongo.humidityMMAR(start, end)
                if val:
                    return jsonify({"status":"found","data":val})
            
            except Exception as e:
                msg = str(e)
                print(f"get_all error: f{str(e)}")

            return jsonify({"status":"not found","data":[]})



# 3. CREATE ROUTE FOR 'pressure'
@app.route('/api/mmar/pressure/<start>/<end>', methods=['GET'])
def getpressureMMAR(start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR PRESSURE. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        start= int(start)
        end= int(end)
        print(f"Start Date: {start}")
        print(f"End Date: {end}")

        if request.method == "GET":
            try:
                val= mongo.pressureMMAR(start, end)
                if val:
                    return jsonify({"status":"found","data":val})
            
            except Exception as e:
                msg = str(e)
                print(f"get_all error: f{str(e)}")

            return jsonify({"status":"not found","data":[]})

# 4. CREATE ROUTE FOR 'altitude'
@app.route('/api/mmar/altitude/<start>/<end>', methods=['GET'])
def getaltitudeMMAR(start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR HUMIDITY. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        start= int(start)
        end= int(end)

        if request.method == "GET":
            try:
                val= mongo.altitudeMMAR(start, end)
                if val:
                    return jsonify({"status":"found","data":val})
            
            except Exception as e:
                msg = str(e)
                print(f"get_all error: f{str(e)}")

            return jsonify({"status":"not found","data":[]})

# 5. CREATE ROUTE FOR 'soil moisture'
@app.route('/api/mmar/soilmoisture/<start>/<end>', methods=['GET'])
def getsoilmoistureMMAR(start, end):
        '''RETURNS MIN, MAX, AVG AND RANGE FOR SOIL MOISTURE. THAT FALLS WITHIN THE START AND END DATE RANGE'''
        start= int(start)
        end= int(end)

        if request.method == "GET":
            try:
                val= mongo.soilmoistureMMAR(start, end)
                if val:
                    return jsonify({"status":"found","data":val})
            
            except Exception as e:
                msg = str(e)
                print(f"get_all error: f{str(e)}")

            return jsonify({"status":"not found","data":[]})

# 6. SERVE FILE FROM UPLOADS DIRECTORY
@app.route('/api/weatherstation/get/<start>/<end>', methods=['GET']) 
def get_all(start,end):   
    '''RETURNS ALL THE DATA FROM THE DATABASE THAT EXIST IN BETWEEN THE START AND END TIMESTAMPS'''
    start= int(start)
    end= int(end)
   
    if request.method == "GET":
        try:
            val= mongo.getAllInRange(start, end)
        
            if val:
                return jsonify({"status":"found","data":val})
        
        except Exception as e:
            msg = str(e)
            print(f"get_all error: f{str(e)}")

    return jsonify({"status":"not found","data":[]})



# 7. UPLOAD FILE TO UPLOADS DIRECTORY
@app.route('/api/frequency/<variable>/<start>/<end>', methods=['GET'])
def getfrequencyDistro(variable,start, end):
    '''RETURNS THE FREQUENCY DISTROBUTION FOR A SPECIFIED VARIABLE WITHIN THE START AND END DATE RANGE'''
    start= int(start)
    end= int(end)
    
    if request.method == "GET":
        try:
            val= mongo.frequencyDistro(variable, start, end)
        
            if val:
                return jsonify({"status":"found","data":val})
        
        except Exception as e:
            msg = str(e)
            print(f"get_all error: f{str(e)}")

    return jsonify({"status":"not found","data":[]})


@app.route('/api/file/get/<filename>', methods=['GET']) 
def get_images(filename):   
    '''Returns requested file from uploads folder'''
   
    if request.method == "GET":
        directory   = join( getcwd(), Config.UPLOADS_FOLDER) 
        filePath    = join( getcwd(), Config.UPLOADS_FOLDER, filename) 

        # RETURN FILE IF IT EXISTS IN FOLDER
        if exists(filePath):        
            return send_from_directory(directory, filename)
        
        # FILE DOES NOT EXIST
        return jsonify({"status":"file not found"}), 404


@app.route('/api/file/upload',methods=["POST"])  
def upload():
    '''Saves a file to the uploads folder'''
    
    if request.method == "POST": 
        file     = request.files['file']
        filename = secure_filename(file.filename)
        file.save(join(getcwd(),Config.UPLOADS_FOLDER , filename))
        return jsonify({"status":"File upload successful", "filename":f"{filename}" })

# POST DATA TO MONGO DATABASE 
@app.route('/api/update', methods=['POST']) 
def update_weather():      
    if request.method == "POST":
        try:
            jsonDoc= request.get_json()
            print("Received JSON:", jsonDoc)  # Debug log
            # Update the document in the 'weatherstation' collection with the new data
            jsonDoc['Temperature'] = jsonDoc.get('celsTemperature', jsonDoc.get('bmp_temp'))
            jsonDoc['Farenheit'] = jsonDoc.get('fahrTemperature')
            jsonDoc['Humidity'] = jsonDoc.get('humidity')
            jsonDoc['Heat Index C'] = jsonDoc.get('heatindex')
            jsonDoc['Soil Moisture'] = jsonDoc.get('soilMoisture')
            jsonDoc['Pressure'] = jsonDoc.get('pressure')
            jsonDoc['Altitude'] = jsonDoc.get('altitude')

            timestamp = datetime.now().timestamp()
            timestamp = floor(timestamp)
            jsonDoc['timestamp'] = timestamp

            Mqtt.publish("620161390",mongo.dumps(jsonDoc))
            Mqtt.publish("620161390_pub",mongo.dumps(jsonDoc))
            Mqtt.publish("620161390_sub",mongo.dumps(jsonDoc))

            print(f"MQTT: {jsonDoc}")

            item = mongo.addUpdate(jsonDoc)
            if item:
                return jsonify({"status": "complete", "data": "complete"})
        except Exception as e:
            msg = str(e)
            print(f"update Error: {msg}")
        return jsonify({"status": "failed", "data": "failed"})





@app.route('/<file_name>.txt')
def send_text_file(file_name):
    """Send your static text file."""
    file_dot_text = file_name + '.txt'
    return app.send_static_file(file_dot_text)

# GLOBAL FLASK SETTINGS
@app.after_request
def add_header(response):
    """Forces latest rendering engine and prevents caching"""
    response.headers['X-UA-Compatible'] = 'IE=Edge,chrome=1'
    response.headers['Cache-Control'] = 'public, max-age=0'
    return response

@app.errorhandler(404)
def page_not_found(error):
    """Custom 404 response"""
    return jsonify({"status": 404}), 404
