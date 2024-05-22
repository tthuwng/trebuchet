import numpy as np
from flask import Flask, jsonify, render_template, request

from trebuchet import trebuchet

app = Flask(__name__)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/simulate", methods=["POST"])
def simulate():
    params = request.json
    NP = 2**12
    i = np.linspace(1, NP, NP)
    total_time = 10
    dt = total_time / NP
    t = i * dt

    xm, ym, angle_theta, t = trebuchet(NP, t, dt, params)

    return jsonify(
        {
            "x": xm.tolist(),
            "y": ym.tolist(),
            "angle_theta": angle_theta.tolist(),
            "t": t.tolist(),
        }
    )


if __name__ == "__main__":
    app.run(debug=True)
