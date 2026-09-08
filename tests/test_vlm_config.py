import os
import unittest
from unittest.mock import patch

from g1_grasp.vlm import VLMSettings


class VLMConfigTests(unittest.TestCase):
    def test_missing_key_fails_before_client_creation(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(RuntimeError, "AIHUBMIX_API_KEY"):
                VLMSettings.from_env()

    def test_key_comes_from_environment(self):
        with patch.dict(os.environ, {"AIHUBMIX_API_KEY": "test-placeholder"}, clear=True):
            settings = VLMSettings.from_env()
        self.assertEqual(settings.api_key, "test-placeholder")
        self.assertEqual(settings.model, "gpt-5.6-terra")


if __name__ == "__main__":
    unittest.main()
