"""Vehicle-VM diagnostic registration via package.preload; no game-file edits."""
import json
from pathlib import Path
import re

NAME = 'wildosContactAudit20261018'
PROTOCOL = 'wildos_contact_audit_v1'


def literal(instance):
    if not isinstance(instance, str) or not re.fullmatch(r'[a-z0-9_]{1,64}', instance):
        raise ValueError('Invalid diagnostic instance identifier')
    return json.dumps(instance)


def install(instance):
    ident = literal(instance)
    source = Path(__file__).with_name('contact_observer.lua').read_text()
    return f"""
local name={json.dumps(NAME)}
local key='controller/'..name
local ownerKey=key..'__auditOwner'
assert(controller.getController(name)==nil, 'Diagnostic controller already present')
assert(package.preload[key]==nil and package.loaded[key]==nil, 'Diagnostic module already present')
assert(package.loaded[ownerKey]==nil, 'Diagnostic registration already owned')
package.loaded[ownerKey]={{instance_id={ident},protocol={json.dumps(PROTOCOL)}}}
package.preload[key]=function()
{source}
end
local c=controller.loadControllerExternal(name,name,{{instance_id={ident},manualOrder=2000000000}})
assert(c and c.protocol=={json.dumps(PROTOCOL)} and c.instance_id=={ident}, 'Diagnostic load failed')
return jsonEncode({{protocol=c.protocol,instance_id=c.instance_id,installed=true}})
"""


def snapshot(instance):
    ident = literal(instance)
    return f"""
local c=assert(controller.getController({json.dumps(NAME)}),'Diagnostic controller missing')
assert(c.protocol=={json.dumps(PROTOCOL)} and c.instance_id=={ident},'Diagnostic owner mismatch')
return jsonEncode(c.snapshot())
"""


def uninstall(instance):
    ident = literal(instance)
    return f"""
local name={json.dumps(NAME)}
local key='controller/'..name
local ownerKey=key..'__auditOwner'
local owner=package.loaded[ownerKey]
local c=controller.getController(name)
if owner==nil then
  return jsonEncode({{instance_id={ident},absent=true,
    unloaded=c==nil and package.loaded[key]==nil and package.preload[key]==nil}})
end
assert(owner.instance_id=={ident} and owner.protocol=={json.dumps(PROTOCOL)},'Diagnostic registration owner mismatch')
if c then
  assert(c.protocol=={json.dumps(PROTOCOL)} and c.instance_id=={ident},'Diagnostic owner mismatch')
  controller.unloadControllerExternal(name)
end
assert(controller.getController(name)==nil,'Diagnostic unload failed')
package.loaded[key]=nil
package.preload[key]=nil
package.loaded[ownerKey]=nil
return jsonEncode({{instance_id={ident},unloaded=true}})
"""
