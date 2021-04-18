const rclnodejs = require('rclnodejs');

rclnodejs.init().then(() => {
    const node = rclnodejs.createNode('subscription_example_node');

    node.createSubscription('std_msgs/msg/String', 'topic', (msg) => {
        console.log(`Received message: ${typeof msg}`, msg);
    });

    node.spin();
});