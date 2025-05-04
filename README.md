# HelloRIC UI Communication
Connects the UI (websocket) with the backend-LLM via ROS 2 messages.

```mermaid
sequenceDiagram
    Browser-->>+websocket: connects
    Note left of websocket: msg.add generates unique_id for new client
    websocket->>+Browser: {emotion: 0, speaking: False} (new_client)
    Browser->>+websocket: {'audio_data': ...}
    websocket->>+ROS: /microphone:String (audio Publisher)
    ROS->>ROS: LLM
    ROS->>+websocket: /websocket/play_audio:PlayAudio (audio_receiver Subscriber)
    websocket->>+Browser: {'text', 'b64audio', 'is_pause', 'is_move', 'emotion'}

````
