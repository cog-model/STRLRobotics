import pytest
from llm_sorter.processors.utils import Conversation


@pytest.fixture
def conversation():
    conv = Conversation(
        name="vicuna_v1.1",
        system_message="A chat between a curious user and an artificial intelligence assistant. "
        "The assistant gives helpful, detailed, and polite answers to the user's questions.",
        roles=("USER", "ASSISTANT"),
        sep=" ",
        sep2="</s>",
    )
    return conv


class TestConversation:
    def test_system_message_output(self, conversation):
        assert (
            conversation.get_prompt()
            == "A chat between a curious user and an artificial intelligence assistant. The assistant gives helpful, detailed, and polite answers to the user's questions. "
        )

    def test_user_message_output(self, conversation):
        conversation.append_message(conversation.roles[0], "Hello")
        assert (
            conversation.get_prompt()
            == "A chat between a curious user and an artificial intelligence assistant. The assistant gives helpful, detailed, and polite answers to the user's questions. USER: Hello "
        )
