"""App artifact and immutable CLI regressions. No network/device/ROS init."""
import ast
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from services.candidate_build import build, replace_once
from services.packet_archive import source_packet, export_packet, verify_archive, parse_manifest
APP=Path(__file__).resolve().parents[1]
ROOT=APP.parents[1]
SOURCE=ROOT/'releases/r5_20261029/ros'

class EntryTests(unittest.TestCase):
    def test_parent_364_unchanged_and_generated_entrypoints_not_named_r5(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest=Path(tmp)/'app'; packet=build(SOURCE,APP,dest)
            raw,entries=source_packet(dest)
            self.assertEqual(packet,hashlib.sha256(raw).hexdigest())
            for line in (SOURCE/'operational_packet.sha256').read_text().splitlines():
                _,name=line.split('  ',1)
                self.assertEqual((SOURCE/name).read_bytes(),(dest/name).read_bytes(),name)
            for name in ('app_fire_entry.py','app_pi_entry.py'):
                ast.parse((dest/'desktop/generated'/name).read_text())
            fire=(dest/'desktop/generated/app_fire_entry.py').read_text()
            self.assertIn('prepare_start_monitor',fire)
            self.assertIn('handoff_monitor',fire)
            self.assertIn('from services.app_peer import AppPeer',fire)
            self.assertNotIn("ros('r5_startup_health'",fire)
            archive=Path(tmp)/'source.tar.gz';export_packet(dest,archive)
            self.assertEqual(verify_archive(archive,packet)['source_pins'],len(entries))
            with self.assertRaises(FileExistsError):export_packet(dest,archive)
            (dest/'desktop/services/display_guard.py').write_text('changed')
            with self.assertRaises(ValueError):source_packet(dest)
            with self.assertRaises(ValueError):build(SOURCE,APP,dest)
    def test_tampered_parent_and_missing_duplicate_anchor_refused(self):
        for text in ('none','xx xx'):
            with self.assertRaises(ValueError):replace_once(text,'xx','yy')
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); (root/'operational_packet.sha256').write_text('changed')
            with self.assertRaisesRegex(ValueError,'UNREVIEWED_R5_PARENT'):build(root,APP,root.parent/(root.name+'_dest'))
            self.assertFalse((root.parent/(root.name+'_dest')).exists())
        with self.assertRaisesRegex(ValueError,'APP_OUTPUT_INSIDE_IMMUTABLE_R5'):
            build(SOURCE,APP,SOURCE/'.qa_app_output_must_not_exist')
        self.assertFalse((SOURCE/'.qa_app_output_must_not_exist').exists())
    def test_archive_manifest_rejects_paths_cache_duplicate_and_symlink_members(self):
        for name in ('../bad','/bad','desktop/../bad','desktop/__pycache__/bad','other/file','desktop/a\\b'):
            with self.subTest(name=name),self.assertRaises(ValueError):parse_manifest(('a'*64+'  '+name+'\n').encode())
        with self.assertRaises(ValueError):parse_manifest((('a'*64+'  desktop/a\n')*2).encode())
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'bad.tar.gz'
            with tarfile.open(p,'w:gz') as tar:
                row=tarfile.TarInfo('operational_packet.sha256');row.type=tarfile.SYMTYPE;row.linkname='/etc/passwd';tar.addfile(row)
            with self.assertRaises(ValueError):verify_archive(p,'a'*64)
    def test_cli_default_still_plan_only_original_sources(self):
        result=subprocess.run([sys.executable,'-B',str(SOURCE/'scripts/r5_laptop_session.py')],
                              capture_output=True,text=True,timeout=15)
        self.assertEqual(result.returncode,0,result.stderr)
        row=json.loads(result.stdout)
        self.assertEqual(row['mode'],'PLAN_ONLY_LIVE_FALSE')
        self.assertEqual(row['commands']['rviz'][0],'rviz2')
    def test_launcher_build_does_not_select_current_or_change_r5(self):
        prepare=(APP/'launchers/prepare_candidate.sh').read_text()
        self.assertNotIn('mv -f',prepare)
        self.assertNotIn('> "$app_dir/artifacts/.current',prepare)
        sync=(APP/'launchers/sync_pi_candidate.sh').read_text()
        self.assertNotIn('python3 scripts/r5_packet.py',sync)
        self.assertIn('services.packet_archive',sync)
        self.assertNotIn('/home/pi/r5_staging/',sync)
