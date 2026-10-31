"""Check that both local project instruction entry points retain the user's rules."""

from pathlib import Path
import unittest


PROJECT = Path(__file__).resolve().parents[1]
HEADING = "## 改动提交与测试要求（2026-10-11 更新）"


class AgentPolicyTest(unittest.TestCase):
    def policy(self, workspace):
        content = (workspace / "AGENTS.md").read_text(encoding="utf-8")
        self.assertEqual(content.count(HEADING), 1)
        return content.split(HEADING, 1)[1].strip()

    def assert_requirements(self, policy):
        for requirement in (
            "只有本次改动所要求的测试和验证全部成功时，才创建对应的 Git commit",
            "测试失败、未执行或必要验收待验证时不提交",
            "保留工作区改动、失败证据和待验证记录",
            "通过后再提交",
            "不得夹带无关改动",
            "交付时提供 commit ID",
            "每次改动后，必须编写或更新相关测试",
            "确保相关测试和验证全部通过",
            "不得宣称修复完成",
            "离线通过不能替代必需的现场验收",
        ):
            self.assertIn(requirement, policy)
        self.assertNotIn("每次改动后必须创建对应的 Git commit", policy)

    def test_primary_workspace_requirements(self):
        self.assert_requirements(self.policy(PROJECT))

    def test_auxiliary_workspace_requirements(self):
        self.assert_requirements(self.policy(PROJECT.parent / "Beamng"))

    def test_workspace_rules_are_consistent(self):
        self.assertEqual(self.policy(PROJECT), self.policy(PROJECT.parent / "Beamng"))


if __name__ == "__main__":
    unittest.main()
