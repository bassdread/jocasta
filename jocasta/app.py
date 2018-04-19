from flask import Flask
from flask import render_template
from data_providers.serial.data_readers import DataSensor
import typing as t
app = Flask(__name__)


SENSOR = DataSensor()


@app.route('/light')
def light():
    reading = SENSOR.read()
    return render_template('light.html', reading=reading)


@app.route('/humidity')
def humidity():
    reading = SENSOR.read()
    return render_template('humidity.html', reading=reading)


@app.route('/temperature')
def temperature():
    reading = SENSOR.read()
    return render_template('temperature.html', reading=reading)


@app.route('/')
def index():
    reading = SENSOR.read()
    return render_template('index.html', reading=reading)


if __name__ == "__main__":
    app.run()
