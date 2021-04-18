// Imports the Google Cloud client library
const {PubSub} = require('@google-cloud/pubsub');
let projectId = "fes-pub-sub";
const pubsub = new PubSub({projectId});

module.exports = {
    publishMessage: async function(topicName, data) {
        // Publishes the message as a string, e.g. "Hello, world!" or JSON.stringify(someObject)
        const dataBuffer = Buffer.from(JSON.stringify(data));

        try {
            const messageId = await pubsub.topic(topicName).publish(dataBuffer);
            console.log(`${messageId} SENT ${dataBuffer} ON ${topicName}`);
        } catch (error) {
            console.error(`Received error while publishing: ${error.message}`);
            process.exitCode = 1;
        }
    }
}
//
// async function quickstart(
//     projectId = 'fes-pub-sub', // Your Google Cloud Platform project ID
//     topicName = 'my-topic', // Name for the new topic to create
//     subscriptionName = 'my-sub' // Name for the new subscription to create
// ) {
    // Instantiates a client

    // Creates a new topic
    // const [topic] = await pubsub.createTopic(topicName);
    // console.log(`Topic ${topic.name} created.`);

    // Creates a subscription on that new topic
    // const [subscription] = await topic.createSubscription(subscriptionName);

    // Receive callbacks for new messages on the subscription
    // subscription.on('message', message => {
    //     console.log('Received message:', message.data.toString());
    //     process.exit(0);
    // });

    // Receive callbacks for errors on the subscription
    // subscription.on('error', error => {
    //     console.error('Received error:', error);
    //     process.exit(1);
    // });

    // Send a message to the topic
    // topic.publish(Buffer.from('Test message!'));
// }