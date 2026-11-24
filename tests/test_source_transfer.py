import json
import numpy as np
import pytest
from contact_transfer.source_transfer import source_fixture,load_source,refine_source_and_transfer


def test_small_scale_positions_are_refined_and_only_geometry_transfers():
    before=refine_source_and_transfer(enabled=False);after=refine_source_and_transfer()
    assert after['synthetic_input'] and not after['inference_executed']
    assert after['source_certificate'] and after['source_validation_residual']<1e-6
    assert after['source_rho_after']>after['source_rho_before']+.01
    assert np.max(abs(np.array(after['source_positions_after_m'])-after['source_positions_before_m']))>.001
    np.testing.assert_allclose(before['source_positions_after_m'],before['source_positions_before_m'],atol=1e-7)
    np.testing.assert_allclose(after['target_positions_m'],4*np.array(after['source_positions_after_m']))
    assert not after['source_forces_transferred'] and not after['target_mass_reused_from_source']
    assert after['source_object']['mass']==.03125


@pytest.mark.parametrize('mutation',[lambda d:d.update(contact_frame='world'),lambda d:d['groups'][1].update(group_id='edge_grip'),lambda d:d['groups'][0].update(position=[-.08,0,0]),lambda d:d['groups'][0].update(normal=[0,0,1])])
def test_source_requires_consistent_surface_groups(tmp_path,mutation):
    d=source_fixture();mutation(d);p=tmp_path/'source.json';p.write_text(json.dumps(d))
    with pytest.raises(ValueError):load_source(p)
