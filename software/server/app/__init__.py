from flask import Flask
import os
import logging


def create_app(test_config=None):
    # create and configure the app
    app = Flask(__name__, instance_relative_config=True)

    log = logging.getLogger('werkzeug')
    # comment out if server debug required
    log.setLevel(logging.ERROR)
    
    from app.tracker import trackerRuntime
    app.register_blueprint(trackerRuntime.bp)

    from app.unity import unityRuntime
    app.register_blueprint(unityRuntime.bp)


    #curl -v http://127.0.0.1:5000/init-test
    @app.route('/init-test')
    def initTest():
        return 'App is created and this route is registered!'

    @app.route('/json-test')
    def jsonTest():
        return {
        "obj1":1,
        "obj2":2
        }

    return app