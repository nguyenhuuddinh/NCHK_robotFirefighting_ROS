"""Small diagnostic allowlist; never record passwords, pairing, full env/argv."""
import os

KEYS=('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','RMW_FASTRTPS_USE_QOS_FROM_XML',
      'FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','RMW_FASTRTPS_PUBLICATION_MODE')

def record(out, root, role, write):
    write(out/'app_environment.json',dict(schema_version=1,role=role,candidate_root=str(root),
        environment={key:os.environ.get(key,'')[:1024] for key in KEYS},
        scope='DIAGNOSTIC_ALLOWLIST_NOT_READINESS_PROOF'))
