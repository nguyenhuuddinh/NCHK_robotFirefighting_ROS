import tempfile
import unittest
from pathlib import Path

from services.map_catalog import InvalidMap, catalog, still_matches, validate_map


class MapCatalogTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="robot maps ")
        self.root = Path(self.tmp.name)
        self.image = self.root / "map_1.pgm"
        self.yaml = self.root / "map_1.yaml"
        # The first pixel deliberately is whitespace to catch raster parsing.
        self.image.write_bytes(b"P5\n# map\n2 2\n255\n\x0a\x20\x00\xff")
        self.yaml.write_text(
            "image: map_1.pgm\nresolution: 0.05\norigin: [0, -1, 0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

    def tearDown(self):
        self.tmp.cleanup()

    def test_catalog_valid_and_revalidate_changed_image(self):
        valid, invalid = catalog([self.root])
        self.assertEqual(invalid, [])
        self.assertEqual(len(valid), 1)
        self.assertEqual((valid[0].width, valid[0].height), (2, 2))
        self.assertTrue(still_matches(valid[0], [self.root]))
        self.image.write_bytes(b"P5\n2 2\n255\n\x00\x00\x00\x00")
        self.assertFalse(still_matches(valid[0], [self.root]))

    def test_missing_or_truncated_image_does_not_produce_map(self):
        self.image.write_bytes(b"P5\n2 2\n255\n\x01")
        valid, invalid = catalog([self.root])
        self.assertEqual(valid, [])
        self.assertIn("raster size", invalid[0]["reason"])
        self.image.unlink()
        with self.assertRaises(InvalidMap):
            validate_map(self.yaml, [self.root])

    def test_rejects_invalid_metadata_and_unsafe_yaml(self):
        base = self.yaml.read_text()
        for changed in (
            base.replace("resolution: 0.05", "resolution: 0"),
            base.replace("origin: [0, -1, 0]", "origin: [0, .nan, 0]"),
            base.replace("occupied_thresh: 0.65", "occupied_thresh: 0.1"),
            base + "image: alternate.pgm\n",
            base + "resolution: 1.0\n",
            "!!python/object/apply:os.system ['false']\n",
        ):
            self.yaml.write_text(changed)
            with self.subTest(yaml=changed), self.assertRaises(InvalidMap):
                validate_map(self.yaml, [self.root])

    def test_rejects_image_path_outside_root_and_invalid_filename(self):
        self.yaml.write_text(self.yaml.read_text().replace("map_1.pgm", "../escape.pgm"))
        with self.assertRaises(InvalidMap):
            validate_map(self.yaml, [self.root])
        with self.assertRaises(InvalidMap):
            validate_map(self.root / "bad name.yaml", [self.root])

    def test_duplicate_names_from_distinct_roots_keep_provenance(self):
        second = self.root / "other"
        second.mkdir()
        (second / self.image.name).write_bytes(self.image.read_bytes())
        (second / self.yaml.name).write_text(self.yaml.read_text())
        valid, invalid = catalog([self.root, second])
        self.assertEqual(invalid, [])
        self.assertEqual(len(valid), 2)
        self.assertNotEqual(valid[0].yaml_path, valid[1].yaml_path)


if __name__ == "__main__":
    unittest.main()
