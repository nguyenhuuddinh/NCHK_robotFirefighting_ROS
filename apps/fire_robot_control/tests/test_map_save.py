import tempfile
import unittest
from pathlib import Path

from services.map_catalog import still_matches
from services.map_save import MapSaveError, save_map_pair


def generate(prefix):
    prefix.with_suffix(".pgm").write_bytes(b"P5\n2 2\n255\n\x00\x00\xff\xff")
    prefix.with_suffix(".yaml").write_text(
        f"image: {prefix.name}.pgm\nmode: trinary\nresolution: 0.05\n"
        "origin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


class MapSaveTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="map save ")
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_complete_map_validated_and_selected_by_caller(self):
        proofs = []
        result = save_map_pair(self.root, "new_map", assert_owned_mapping=lambda: proofs.append(True),
                               generate=generate)
        self.assertGreaterEqual(len(proofs), 5)
        self.assertTrue(still_matches(result, [self.root]))
        self.assertEqual(sorted(p.name for p in self.root.iterdir()), ["new_map.pgm", "new_map.yaml"])

    def test_existing_map_and_invalid_names_never_overwrite(self):
        original = self.root / "old.pgm"
        original.write_bytes(b"user content")
        for name in ("old", "../map", "map;touch", "a" * 65):
            with self.subTest(name=name), self.assertRaises(MapSaveError):
                save_map_pair(self.root, name, assert_owned_mapping=lambda: None, generate=generate)
        self.assertEqual(original.read_bytes(), b"user content")

    def test_failed_generator_or_lost_slam_never_leaves_final_pair(self):
        def incomplete(prefix):
            prefix.with_suffix(".pgm").write_bytes(b"incomplete")
        with self.assertRaises(MapSaveError):
            save_map_pair(self.root, "new", assert_owned_mapping=lambda: None, generate=incomplete)
        self.assertEqual(list(self.root.iterdir()), [])
        calls = []
        def proof():
            calls.append(1)
            if len(calls) == 2:
                raise MapSaveError("SLAM no longer owned")
        with self.assertRaises(MapSaveError):
            save_map_pair(self.root, "new", assert_owned_mapping=proof, generate=generate)
        self.assertEqual(list(self.root.iterdir()), [])

    def test_racing_existing_yaml_preserves_user_file_and_rolls_back_own_image(self):
        def race(prefix):
            generate(prefix)
            (self.root / "new.yaml").write_text("user file")
        with self.assertRaises(MapSaveError):
            save_map_pair(self.root, "new", assert_owned_mapping=lambda: None, generate=race)
        self.assertEqual((self.root / "new.yaml").read_text(), "user file")
        self.assertFalse((self.root / "new.pgm").exists())

    def test_stop_during_commit_rolls_back_the_partial_pair(self):
        checks = []
        def proof():
            checks.append(True)
            if len(checks) == 4:
                raise MapSaveError("stop during commit")
        with self.assertRaises(MapSaveError):
            save_map_pair(self.root, "new", assert_owned_mapping=proof, generate=generate)
        self.assertFalse((self.root / "new.yaml").exists())
        self.assertFalse((self.root / "new.pgm").exists())


if __name__ == "__main__":
    unittest.main()
