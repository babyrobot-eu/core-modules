import json
import Queue
import time
import threading
import urllib
from ws4py.client.threadedclient import WebSocketClient
import rospy


def rate_limited(max_per_second):
    min_interval = 1.0 / float(max_per_second)

    def decorate(func):
        last_time_called = [0.0]

        def rate_limited_function(*args, **kargs):
            elapsed = time.clock() - last_time_called[0]
            left_to_wait = min_interval - elapsed
            if left_to_wait > 0:
                time.sleep(left_to_wait)
            ret = func(*args, **kargs)
            last_time_called[0] = time.clock()
            return ret
        return rate_limited_function
    return decorate


class MyClient(WebSocketClient):
    def __init__(self, audiofile, url, protocols=None, extensions=None,
                 heartbeat_freq=None, byterate=32000,
                 save_adaptation_state_filename=None,
                 send_adaptation_state_filename=None):
        super(MyClient, self).__init__(
            url, protocols, extensions, heartbeat_freq)
        self.final_hyps = []
        self.audiofile = audiofile
        self.byterate = byterate
        self.final_hyp_queue = Queue.Queue()
        self.save_adaptation_state_filename = \
            save_adaptation_state_filename
        self.send_adaptation_state_filename = \
            send_adaptation_state_filename

    @rate_limited(4)
    def send_data(self, data):
        self.send(data, binary=True)

    def opened(self):
        def send_data_to_ws():
            if self.send_adaptation_state_filename is not None:
                rospy.logerr("Sending adaptation state from {}".format(
                    self.send_adaptation_state_filename))
                try:
                    adaptation_state_props = json.load(
                        open(self.send_adaptation_state_filename, "r"))
                    self.send(json.dumps(
                        dict(adaptation_state=adaptation_state_props)))
                except Exception, ex:
                    rospy.logerr("Failed to send adaptation state: {}"
                                 .format(ex))
            with self.audiofile as audiostream:
                for block in iter(lambda: audiostream.read(
                        self.byterate/4), ""):
                    self.send_data(block)
            rospy.logerr("Audio sent, now sending EOS")
            self.send("EOS")

        t = threading.Thread(target=send_data_to_ws)
        t.start()

    def received_message(self, m):
        response = json.loads(str(m))
        if response['status'] == 0:
            if 'result' in response:
                trans = response['result']['hypotheses'][0]['transcript']
                if response['result']['final']:
                    self.final_hyps.append(trans)
                    rospy.logerr('\r%s' % trans.replace("\n", "\\n"))
                else:
                    print_trans = trans.replace("\n", "\\n")
                    if len(print_trans) > 80:
                        print_trans = "... %s" % print_trans[-76:]
                    rospy.logerr('\r%s' % print_trans)
            if 'adaptation_state' in response:
                if self.save_adaptation_state_filename:
                    rospy.logerr("Saving adaptation state to {}".format(
                        self.save_adaptation_state_filename))
                    with open(self.save_adaptation_state_filename, "w") as f:
                        f.write(json.dumps(response['adaptation_state']))
        else:
            rospy.logerr("Received error from server (status {})".format(
                response['status']))
            if 'message' in response:
                rospy.logerr("Error message: {}".format(response['message']))

    def get_full_hyp(self, timeout=60):
        return self.final_hyp_queue.get(timeout)

    def closed(self, code, reason=None):
        self.final_hyp_queue.put(" ".join(self.final_hyps))


def run_gst(server_url, clip_path, byte_rate=32000):
    clip = open(clip_path, 'r')
    content_type = ''
    request_url = "{0}{1}".format(
        server_url,
        urllib.urlencode([("content-type", content_type)]))
    ws = MyClient(
        clip,
        request_url,
        byterate=byte_rate)
    ws.connect()
    result = ws.get_full_hyp()
    return result.encode('utf-8')
