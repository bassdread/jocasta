from flask import Flask
from flask import render_template
from data_providers.serial.data_readers import DataSensor
import typing as t
app = Flask(__name__)


SENSOR = DataSensor()

@app.route('/')
def hello():
    reading = SENSOR.read()
    return render_template('index.html', reading=reading)


if __name__ == "__main__":
    app.run()
