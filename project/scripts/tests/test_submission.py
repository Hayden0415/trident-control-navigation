"""Host-side regression tests for submission boundaries and ZIP integrity."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from submit import package, source_files


class PackagingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)/'project'; self.root.mkdir()
        subprocess.run(['git', 'init', '-q', str(self.root)], check=True)
        (self.root/'src').mkdir(); (self.root/'src/answer.py').write_text('original\n')
        (self.root/'README.md').write_text('assignment\n')
        subprocess.run(['git', '-C', str(self.root), 'add', '.'], check=True)
        (self.root/'src/answer.py').write_text('working tree answer\n')
        (self.root/'src/extra.py').write_text('new helper\n')
        for folder in ('.vscode', 'build', 'src/__pycache__'):
            (self.root/folder).mkdir(parents=True, exist_ok=True)
            (self.root/folder/'private').write_text('must not ship')
        self.output = Path(self.temp.name)/'run'; self.output.mkdir()
        (self.output/'result.json').write_text('{"passed": false}')
        self.report = Path(self.temp.name)/'report.md'; self.report.write_text('student report')

    def tearDown(self): self.temp.cleanup()

    def test_working_tree_new_helpers_evidence_and_checksums(self):
        archive = Path(self.temp.name)/'submission.zip'
        package(self.root, self.output, self.report, archive)
        with zipfile.ZipFile(archive) as z:
            self.assertEqual(z.read('project/src/answer.py'), b'working tree answer\n')
            self.assertIn('project/src/extra.py', z.namelist())
            self.assertIn('report.md', z.namelist())
            self.assertIn('evidence/result.json', z.namelist())
            self.assertFalse(any('/.git/' in x or '/.vscode/' in x or '/build/' in x or '/__pycache__/' in x for x in z.namelist()))
            for name, expected in json.loads(z.read('SHA256SUMS.json')).items():
                self.assertEqual(hashlib.sha256(z.read(name)).hexdigest(), expected)
            self.assertIsNone(z.testzip())

    def test_symlink_cannot_copy_external_file_into_source(self):
        (self.root/'src/leak.py').symlink_to(self.report)
        with self.assertRaisesRegex(RuntimeError, '符号链接'):
            list(source_files(self.root))


if __name__ == '__main__': unittest.main()
