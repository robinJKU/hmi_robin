#!/usr/bin/env python

# Copyright 2017 Institute of Robotics, Johannes Kepler University Linz (ROS package)
# based on a template by Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from assistant_robin.srv import *
import rospy

import logging
import os.path

import click
import grpc
from google.assistant.embedded.v1alpha1 import embedded_assistant_pb2
from google.rpc import code_pb2
from tenacity import retry, stop_after_attempt, retry_if_exception
from googlesamples.assistant import (
    assistant_helpers,
    audio_helpers,
    auth_helpers,
    common_settings
)


class SampleAssistant(object):
    """Sample Assistant that supports follow-on conversations.

    Args:
      conversation_stream(ConversationStream): audio stream
        for recording query and playing back assistant answer.
      channel: authorized gRPC channel for connection to the
        Google Assistant API.
      deadline_sec: gRPC deadline in seconds for Google Assistant API call.
    """

    def __init__(self, conversation_stream, channel, deadline_sec):
        self.conversation_stream = conversation_stream

        # Opaque blob provided in ConverseResponse that,
        # when provided in a follow-up ConverseRequest,
        # gives the Assistant a context marker within the current state
        # of the multi-Converse()-RPC "conversation".
        # This value, along with MicrophoneMode, supports a more natural
        # "conversation" with the Assistant.
        self.conversation_state = None

        # Create Google Assistant API gRPC client.
        self.assistant = embedded_assistant_pb2.EmbeddedAssistantStub(channel)
        self.deadline = deadline_sec

    def __enter__(self):
        return self

    def __exit__(self, etype, e, traceback):
        if e:
            return False
        self.conversation_stream.close()

    def is_grpc_error_unavailable(e):
        is_grpc_error = isinstance(e, grpc.RpcError)
        if is_grpc_error and (e.code() == grpc.StatusCode.UNAVAILABLE):
            rospy.logerr('grpc unavailable error: %s', e)
            return True
        return False

    @retry(reraise=True, stop=stop_after_attempt(3),
           retry=retry_if_exception(is_grpc_error_unavailable))
    def converse(self):
        """Send a voice request to the Assistant and playback the response.

        Returns: True if conversation should continue.
        """
        continue_conversation = False

        self.conversation_stream.start_recording()
        rospy.logdebug('Recording audio request.')

        def iter_converse_requests():
            for c in self.gen_converse_requests():
                assistant_helpers.log_converse_request_without_audio(c)
                yield c
            self.conversation_stream.start_playback()

        # This generator yields ConverseResponse proto messages
        # received from the gRPC Google Assistant API.
        for resp in self.assistant.Converse(iter_converse_requests(),
                                            self.deadline):
            assistant_helpers.log_converse_response_without_audio(resp)
            if resp.error.code != code_pb2.OK:
                rospy.logerr('server error: %s', resp.error.message)
                break
            if resp.event_type == embedded_assistant_pb2.ConverseResponse.END_OF_UTTERANCE:
                rospy.logdebug('End of audio request detected')
                self.conversation_stream.stop_recording()
            if resp.result.spoken_request_text:
                rospy.logdebug('Transcript of user request: "%s".',
                             resp.result.spoken_request_text)
                rospy.logdebug('Playing assistant response.')
            if len(resp.audio_out.audio_data) > 0:
                self.conversation_stream.write(resp.audio_out.audio_data)
            if resp.result.spoken_response_text:
                rospy.logdebug(
                    'Transcript of TTS response '
                    '(only populated from IFTTT): "%s".',
                    resp.result.spnoken_response_text)
            if resp.result.conversation_state:
                self.conversation_state = resp.result.conversation_state
            if resp.result.volume_percentage != 0:
                self.conversation_stream.volume_percentage = (
                    resp.result.volume_percentage
                )
            if resp.result.microphone_mode == embedded_assistant_pb2.ConverseResult.DIALOG_FOLLOW_ON:
                continue_conversation = True
                rospy.logdebug('Expecting follow-on query from user.')
            elif resp.result.microphone_mode == embedded_assistant_pb2.ConverseResult.CLOSE_MICROPHONE:
                continue_conversation = False
        rospy.logdebug('Finished playing assistant response.')
        self.conversation_stream.stop_playback()
        return continue_conversation

    def gen_converse_requests(self):
        """Yields: ConverseRequest messages to send to the API."""

        converse_state = None
        if self.conversation_state:
            rospy.logdebug('Sending converse_state: %s',
                          self.conversation_state)
            converse_state = embedded_assistant_pb2.ConverseState(
                conversation_state=self.conversation_state,
            )
        config = embedded_assistant_pb2.ConverseConfig(
            audio_in_config=embedded_assistant_pb2.AudioInConfig(
                encoding='LINEAR16',
                sample_rate_hertz=self.conversation_stream.sample_rate,
            ),
            audio_out_config=embedded_assistant_pb2.AudioOutConfig(
                encoding='LINEAR16',
                sample_rate_hertz=self.conversation_stream.sample_rate,
                volume_percentage=self.conversation_stream.volume_percentage,
            ),
            converse_state=converse_state
        )
        # The first ConverseRequest must contain the ConverseConfig
        # and no audio data.
        yield embedded_assistant_pb2.ConverseRequest(config=config)
        for data in self.conversation_stream:
            # Subsequent requests need audio data, but not config.
            yield embedded_assistant_pb2.ConverseRequest(audio_in=data)

def main(*args, **kwargs):

    # init ROS
    rospy.init_node('assistant_robin_server')

    # get parameters
    api_endpoint = rospy.get_param('~api_endpoint', 'embeddedassistant.googleapis.com') # Address of Google Assistant API service.
    credentials = rospy.get_param('~credentials', os.path.join(
                  click.get_app_dir(common_settings.ASSISTANT_APP_NAME),
                  common_settings.ASSISTANT_CREDENTIALS_FILENAME
              )) # Path to read OAuth2 credentials.
    verbose = rospy.get_param('~verbose', False) # Verbose logging.
    audio_sample_rate = rospy.get_param('~audio_sample_rate', common_settings.DEFAULT_AUDIO_SAMPLE_RATE) # Audio sample rate in hertz.
    audio_sample_width = rospy.get_param('~audio_sample_width', common_settings.DEFAULT_AUDIO_SAMPLE_WIDTH) # Audio sample width in bytes.
    audio_iter_size = rospy.get_param('~audio_iter_size', common_settings.DEFAULT_AUDIO_ITER_SIZE) # Size of each read during audio stream iteration in bytes.
    audio_block_size = rospy.get_param('~audio_block_size', common_settings.DEFAULT_AUDIO_DEVICE_BLOCK_SIZE) # Block size in bytes for each audio device read and write operation.
    audio_flush_size = rospy.get_param('~audio_flush_size', common_settings.DEFAULT_AUDIO_DEVICE_FLUSH_SIZE) # Size of silence data in bytes written during flush operation.
    grpc_deadline = rospy.get_param('~grpc_deadline', common_settings.DEFAULT_GRPC_DEADLINE) # gRPC deadline in seconds.


    # Load credentials.
    try:
        creds = auth_helpers.load_credentials(
            credentials, scopes=[common_settings.ASSISTANT_OAUTH_SCOPE]
        )
    except Exception as e:
        rospy.logerr('Error loading credentials: %s', e)
        rospy.logerr('Run auth_helpers to initialize new OAuth2 credentials.')
        return

    # Create an authorized gRPC channel.
    grpc_channel = auth_helpers.create_grpc_channel(
        api_endpoint, creds,
        ssl_credentials_file=kwargs.get('ssl_credentials_for_testing'),
        grpc_channel_options=kwargs.get('grpc_channel_option')
    )
    rospy.logdebug('Connecting to %s', api_endpoint)

    # Configure audio source and sink.
    audio_device = None
    audio_source = audio_device = (
        audio_device or audio_helpers.SoundDeviceStream(
            sample_rate=audio_sample_rate,
            sample_width=audio_sample_width,
            block_size=audio_block_size,
            flush_size=audio_flush_size
        )
    )
    audio_sink = audio_device = (
        audio_device or audio_helpers.SoundDeviceStream(
            sample_rate=audio_sample_rate,
            sample_width=audio_sample_width,
            block_size=audio_block_size,
            flush_size=audio_flush_size
        )
    )
    # Create conversation stream with the given audio source and sink.
    conversation_stream = audio_helpers.ConversationStream(
        source=audio_source,
        sink=audio_sink,
        iter_size=audio_iter_size,
        sample_width=audio_sample_width,
    )

    with SampleAssistant(conversation_stream,
                         grpc_channel, grpc_deadline) as assistant:
        global ass
        ass = assistant
        s = rospy.Service('activate', Activate, handle_activate)
        rospy.loginfo("Assistant ready.")

        rospy.spin()


def handle_activate(req):
    rospy.loginfo("Assistant activated.")
    continue_conversation = True
    while continue_conversation:
        continue_conversation = ass.converse()
    return ActivateResponse()

if __name__ == "__main__":
    main()

