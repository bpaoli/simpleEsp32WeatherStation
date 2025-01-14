from flask import Flask, request
import os

app = Flask(__name__)

# Initialize location variable
location = ""

@app.route('/readfile/location.txt', methods=['GET'])
def read_location_file():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(script_dir, "location.txt")
    with open(file_path, "r") as file:
        location = file.read()
    return location

@app.route('/weather', methods=['POST'])
def update_weather():
    global location
    data = request.data.decode('utf-8')
    print(data)
    # Assuming the data format is "<location>: <outdoor info>\n"
    parts = data.split(':')
    if len(parts) >= 2:
        location = parts[0].strip()
        return f"Location updated to: {location}"
    else:
        return "Invalid data format"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=1234)