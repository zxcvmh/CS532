import json
import os
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SETTINGS = os.path.join(ROOT, ".claude", "settings.json")

# Rule syntax: `Bash(cmd *)` = prefix match with a word boundary (also written without args for the bare command);
# `Edit(path)` / `Write(path)` = gitignore-style path relative to the project. Both forms appear in real configs.
DENY_GIT = ["git add -u", "git add -A", "git add --all", "git add .", "git push", "git reset --hard"]
ASK_REMOTE = ["ssh", "scp", "rsync"]


def rules():
    with open(SETTINGS, encoding="utf-8") as f:
        return json.load(f)["permissions"]


class TestSettingsRules(unittest.TestCase):
    def test_json_valid(self):
        self.assertIn("deny", rules())

    def test_deny_has_destructive_git(self):
        deny = rules()["deny"]
        for cmd in DENY_GIT:
            self.assertIn("Bash(%s)" % cmd, deny)
            self.assertIn("Bash(%s *)" % cmd, deny)

    def test_ask_has_remote_access_and_bridge(self):
        ask = rules().get("ask", [])
        for cmd in ASK_REMOTE:
            self.assertIn("Bash(%s *)" % cmd, ask)
        self.assertIn("Bash(*run_bridge*)", ask)
        self.assertIn("Bash(*jetbot_bridge*)", ask)

    def test_robot_dynamics_still_denied(self):
        deny = rules()["deny"]
        self.assertIn("Edit(setup/robot_dynamics.json)", deny)
        self.assertIn("Write(setup/robot_dynamics.json)", deny)


if __name__ == "__main__":
    unittest.main()
