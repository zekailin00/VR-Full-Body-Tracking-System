import app
import algorithm 


algorithm.intialize()

server = app.create_app()
server.run(host="0.0.0.0", port=5000, threaded=True)

