import rclpy
from std_msgs.msg import String
from flask import *
from flask_cors import CORS

app = Flask(__name__)
app.config["DEBUG"] = True
CORS(app)


@app.route('/api/command/', methods=['GET'])
def get():
    return "<h1>Distant Reading Archive</h1><p>This site is a prototype API for distant reading of science fiction novels.</p>"


@app.route('/hi', methods=['POST'])
def post():
    print(request.data)
    return "thanks!"


class Controller:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node(node_name="controller")
        self.switch_pub = self.node.create_publisher(String, "switch-controller-topic", 10)
        self.mainloop()

    def mainloop(self):
        i = 0
        msg = String()
        while True:
            msg.data = 'Hello World: %d' % i
            i += 1
            self.node.get_logger().info('Publishing: "%s"' % msg.data)
            self.switch_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=1)
            if i == 10:
                break
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    controller = Controller()
    controller.mainloop()
    # app.run()
