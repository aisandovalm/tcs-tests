from flask import Flask
from flask import render_template

app = Flask(__name__)

@app.route("/")
def hello(name=None):
	return render_template('hello.html')

if __name__ == "__main__":
	app.run('0.0.0.0') 
#Esto permite acceder a web server desde cualquier dispositivo en la red
