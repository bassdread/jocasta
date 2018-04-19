from flask import Flask
from flask import render_template
from data_providers.serial.data_readers import DataSensor
import sqlite3
from flask import g
from update_readings import DATABASE
import typing as t
app = Flask(__name__)


SENSOR = DataSensor()


def get_db():
    db = getattr(g, '_database', None)
    if db is None:
        db = g._database = sqlite3.connect(DATABASE)
    return db


@app.teardown_appcontext
def close_connection(exception):
    db = getattr(g, '_database', None)
    if db is not None:
        db.close()


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


@app.route('/db')
def index_db():
    cur = get_db().cursor()
    cur.execute("select * from readings ORDER BY date DESC LIMIT 1;")
    rv = cur.fetchall()[0]
    reading = {
        'humidity': rv[1],
        'light': rv[2],
        'temperature': rv[3],
    }
    return render_template('index.html', reading=reading)


if __name__ == "__main__":
    app.run()
