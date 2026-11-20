"""Unit tests, fake mixer only; no actual device writes."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'control_center_service_manager'))
import speaker_volume as sv


class VolumeTests(unittest.TestCase):
    def test_stereo_muted_or_pcm_zero_is_reported(self):
        self.assertEqual(sv.parse_control('Left [50%] [on]\nRight [49%] [on]')['percent'],49)
        self.assertFalse(sv.parse_control('Left [50%] [off]')['enabled'])
        with patch.object(sv,'mixer',side_effect=['[80%] [on]','[0%]']):
            self.assertEqual(sv.read(4)['volume'],0)

    def test_gain_application_mutes_then_repairs_pcm_then_sets_master(self):
        state={'Master':0,'PCM':0,'enabled':True};calls=[]
        def mixer(card,action,name,*values):
            calls.append((action,name,*values))
            if action=='sget':return f"[{state[name]}%] [{'on' if state['enabled'] or name=='PCM' else 'off'}]"
            for value in values:
                if value.endswith('%'):state[name]=int(value[:-1])
                elif value in ('mute','unmute'):state['enabled']=value=='unmute'
            return ''
        with patch.object(sv,'mixer',side_effect=mixer):
            result=sv.apply(4,60)
            self.assertEqual(result['volume'],60)
            writes=[c for c in calls if c[0]=='sset']
            self.assertEqual(writes,[('sset','Master','mute'),('sset','PCM','100%'),('sset','Master','60%','unmute')])
            self.assertEqual(sv.apply(4,0)['volume'],0)
            self.assertFalse(state['enabled'])

    def test_bad_feedback_is_not_success(self):
        with patch.object(sv,'mixer',return_value='[0%] [on]'):
            with self.assertRaises(RuntimeError):sv.apply(4,60)

    def test_range_check(self):
        for n in (-1,101,True):
            with self.assertRaises(ValueError):sv.apply(4,n)


if __name__=='__main__':unittest.main()
