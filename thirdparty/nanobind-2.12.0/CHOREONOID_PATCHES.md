# nanobind 2.12.0 bundled for Choreonoid — local patches

This directory contains a verbatim copy of upstream nanobind 2.12.0
(https://github.com/wjakob/nanobind, tag `v2.12.0`, released 2025-02-25)
**except** for the small additions documented below. Choreonoid builds this
copy when `CHOREONOID_PYTHON_BINDING_BACKEND=nanobind`.

Upstream nanobind is otherwise meant to be used unmodified. Only the additions
listed here are Choreonoid-local. They are all **pure additions** (new
functions / declarations); no existing upstream logic is edited. Every patched
site is marked with a `[CHOREONOID PATCH]` comment so it can be located with:

```sh
grep -rn "CHOREONOID PATCH" thirdparty/nanobind-2.12.0/
```

When bumping nanobind to a new version, re-apply each patch below into the
freshly extracted upstream tree (the line numbers will differ; match on the
surrounding upstream code shown here, not on line numbers).

---

## Why these patches exist

### The problem: double free of a Python subclass of an `nb::new_()` type

Choreonoid lets Python scripts subclass bound C++ widget types, e.g.

```python
class TrialBar(ToolBar):       # ToolBar is bound with nb::class_ + nb::new_()
    def __init__(self):
        ToolBar.__init__(self, "Trial")

bar = TrialBar()
PythonPlugin.instance.mountToolBar(bar)   # hands ownership of the C++ ToolBar to Qt
```

Since nanobind 2.5.0, subclassing a C++ type whose constructor is bound with
`nb::new_()` is supported. The way nanobind implements it
(`new_returntype_fixup_policy` in `include/nanobind/nb_class.h`, and
`nb_inst_reference()` in `src/nb_type.cpp`) creates **two** Python wrappers for
the **one** C++ object:

1. a hidden base-type wrapper (`cnoid.Base.ToolBar`) that actually **owns** the
   C++ object — its `nb_inst` has `destruct = true`; and
2. the visible subclass wrapper (`__main__.TrialBar`), created with
   `nb_inst_reference()`, which is a **non-owning** reference
   (`destruct = false`) that holds the base wrapper alive via `keep_alive`.

This is fine on its own: when Python drops the subclass wrapper, the keep-alive
chain drops the base wrapper too, and the C++ object is deleted exactly once.

It breaks when ownership of the C++ object is **handed over to C++** (here, Qt:
`mountToolBar()` reparents the tool bar to the main window, which deletes it on
shutdown). Choreonoid releases nanobind's ownership at that boundary via
`cnoid::python::releaseOwnership()` (see `src/Util/python/PyUtil.h`, used by the
`OwnershipReleased<T>` argument caster in the Qt bindings). That release used to
call `nb::inst_set_state(handle, true, false)`, which only clears the
`destruct` flag of the **one** wrapper it is given — the visible subclass
wrapper, which was already non-owning. The hidden owning base wrapper kept
`destruct = true`, so at interpreter finalization nanobind ran the C++
destructor a second time on an object Qt had already deleted →
`QObject: shared QObject was deleted directly` → SIGSEGV.

(For a plain, non-subclassed widget there is only one wrapper and it is the
owner, so the old single-wrapper release worked. The bug only appears for Python
subclasses combined with ownership transfer to C++.)

### The fix

Add a helper that walks nanobind's C++ → Python instance map (`inst_c2p`) and
clears `destruct`/`cpp_delete` on **every** wrapper that refers to the same C++
pointer, including the hidden owner. `releaseOwnership()` calls this instead of
`inst_set_state()`.

This was preferred over editing the upstream `new_returntype_fixup_policy`
(which would change ownership semantics for *all* nanobind subclasses and is far
riskier and harder to carry across version bumps). The helper is an additive,
localized change at the ownership-transfer boundary.

Upstream status (checked 2026-06): nanobind master (post-2.12.0) has no fix for
this subclass-ownership-transfer case; the only relevant nearby work is the
2.12.0 "use-after-free when calling functions after their module has been
deleted" fix, which is unrelated. So this cannot be resolved by simply updating
nanobind.

---

## The patches (re-apply these after a version bump)

### Patch 1 — implementation, `src/nb_type.cpp`

Insert immediately **after** the definition of `nb_inst_set_state(...)** (search
for `void nb_inst_set_state(PyObject *o, bool ready, bool destruct)`):

```cpp
// [CHOREONOID PATCH] See CHOREONOID_PATCHES.md in this directory.
// Clear the destruct/cpp_delete flags of *every* Python wrapper that refers to
// the same C++ object as 'o'. This is needed when ownership of a C++ object is
// handed over to C++ code (e.g. a Qt widget reparented to a Qt parent that will
// delete it). A Python subclass of a bound C++ type created with nb::new_()
// produces two wrappers for one C++ object: the subclass wrapper (which is a
// non-owning reference) and a hidden base-type wrapper that actually owns the
// object (destruct = true). Calling nb_inst_set_state() on only the visible
// (subclass) wrapper leaves the hidden owner with destruct = true, so the C++
// object would be deleted twice. This walks the C++ -> Python instance map and
// relinquishes ownership from all of them.
void nb_cnoid_relinquish_all_wrappers(PyObject *o) noexcept {
    nb_inst *nbi = (nb_inst *) o;
    void *p = inst_ptr(nbi);

    nb_shard &shard = internals->shard(p);
    lock_shard guard(shard);

    nb_ptr_map &inst_c2p = shard.inst_c2p;
    nb_ptr_map::iterator it = inst_c2p.find(p);
    if (it == inst_c2p.end())
        return;

    auto relinquish = [](PyObject *inst) {
        nb_inst *i = (nb_inst *) inst;
        i->destruct = false;
        i->cpp_delete = false;
    };

    void *entry = it->second;
    if (!nb_is_seq(entry)) {
        relinquish((PyObject *) entry);
    } else {
        nb_inst_seq *seq = nb_get_seq(entry);
        while (seq) {
            relinquish(seq->inst);
            seq = seq->next;
        }
    }
}
```

The `inst_c2p` / `nb_shard` / `lock_shard` / `nb_is_seq` / `nb_get_seq` /
`nb_inst_seq` primitives used here are the same ones the upstream
`inst_dealloc()` and `nb_type_put()` use to walk the instance map (see
`src/nb_internals.h`). If a future version renames or restructures these (e.g.
removes the sharding, or changes the seq encoding), mirror whatever
`inst_dealloc()` does to enumerate all instances for a pointer.

### Patch 2 — core declaration, `include/nanobind/nb_lib.h`

Insert immediately **after** the declaration of `nb_inst_state` (search for
`NB_CORE std::pair<bool, bool> nb_inst_state(PyObject *o) noexcept;`):

```cpp
// [CHOREONOID PATCH] See CHOREONOID_PATCHES.md in the nanobind root directory.
// Relinquish ownership (clear destruct/cpp_delete) of every Python wrapper that
// shares the same C++ object as 'o', including the hidden owning base wrapper
// created for a Python subclass of an nb::new_() type.
NB_CORE void nb_cnoid_relinquish_all_wrappers(PyObject *o) noexcept;
```

### Patch 3 — public inline wrapper, `include/nanobind/nb_class.h`

Insert immediately **after** the `inst_state(handle h)` inline (search for
`inline std::pair<bool, bool> inst_state(handle h)`):

```cpp
// [CHOREONOID PATCH] See CHOREONOID_PATCHES.md in the nanobind root directory.
inline void cnoid_relinquish_all_wrappers(handle h) {
    detail::nb_cnoid_relinquish_all_wrappers(h.ptr());
}
```

---

## Consumer in Choreonoid

`src/Util/python/PyUtil.h` — `cnoid::python::releaseOwnership()` calls
`nb::cnoid_relinquish_all_wrappers(obj)`. This is the only caller. It is invoked
through the `OwnershipReleased<T>` argument type caster (same file), which the
Qt/widget bindings use for every method that hands a Python-created object's
ownership to C++ (e.g. `ToolBar.addWidget`, `QBoxLayout.addWidget/addLayout`,
`QWidget.setLayout`, `ExtensionManager.addToolBar/mountToolBar`,
`ViewArea.addView`).

## How to verify after a version bump

1. Rebuild: `cmake --build ~/choreonoid-alt/build-nanobind-ft`
   (a free-threaded python3.13t build with the nanobind backend).
2. Run a minimal repro and quit the app; it must **not** crash on exit:

   ```python
   # /tmp/tbtest.py
   from cnoid.Base import *
   from cnoid.PythonPlugin import *
   class MyBar(ToolBar):
       def __init__(self):
           ToolBar.__init__(self, "MyBar")
           self.addButton("Hello")
   bar = MyBar()
   PythonPlugin.instance.mountToolBar(bar)
   ```
   ```sh
   ./build-nanobind-ft/bin/choreonoid --python /tmp/tbtest.py
   ```
3. Full check: open `share/choreonoid-2.5/seniorcar/project/seniorcar-trials.cnoid`
   (its `seniorcar-trials.py` defines a `ToolBar` subclass `TrialBar` and mounts
   it) and quit. Before this patch the exit crashed in `Py_FinalizeEx()` →
   `inst_dealloc()` → `t->destruct` on the duplicate `cnoid.Base.ToolBar`
   wrapper. With the patch it exits cleanly.

## Related

- Interpreter init/finalize was moved out of PythonPlugin into the CnoidPython
  library (`src/Python/`), finalizing the interpreter as the very last step of
  App shutdown. See `python-finalize-crash-plan.md` in the repository root and
  `src/Python/PythonInterpreter.cpp`. That refactoring fixed the
  Referenced-derived item-tree teardown order; this nanobind patch fixes the
  remaining Qt-widget-subclass double free that the reordering alone did not.
