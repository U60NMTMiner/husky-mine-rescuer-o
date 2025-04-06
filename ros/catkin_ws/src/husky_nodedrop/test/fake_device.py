from flask import Flask, jsonify

app = Flask(__name__)

@app.route('/cgi-bin/luci/iwt/dashboard/get-data')
def mock_response():
    # This is the JSON structure expected by your ROS node
    response_data = {
        "status": "success",
        "neighbors": [
            ["Neighbor1", "info", "info", "info", "info", "info", -85.0],  # Example RSSI
            #["Neighbor2", "info", "info", "info", "info", "info", -65.0]
        ]
    }
    return jsonify(response_data)

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000)  # Mimic a real device on port 5000
