# ige-core::solvers::mic::workspace <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


## Structs

### `ige-core::solvers::mic::workspace::MicCandidate`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">pub(crate)</span>


**Derives:** `Debug`, `Clone`

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `x` | `f64` |  |
| `y` | `f64` |  |
| `radius_sq` | `f64` |  |



### `ige-core::solvers::mic::workspace::MicWorkspace`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Reusable solver workspace for MIC computation.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `host` | `HostPolygon` |  |
| `seg_index` | `SegmentIndex` |  |
| `pip_index` | `PipIndex` |  |
| `nb_index` | `NearestBoundaryIndex` |  |
| `candidate_buf` | `Vec < MicCandidate >` |  |

#### Methods

##### `new` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn new (host : HostPolygon) -> Result < Self , MicError >
```

<details>
<summary>Source</summary>

```rust
    pub fn new(host: HostPolygon) -> Result<Self, MicError> {
        let seg_index = SegmentIndex::from_host(&host);
        if seg_index.is_empty() {
            return Err(MicError::InvalidInput(
                "polygon has no non-degenerate boundary segments".to_string(),
            ));
        }
        let pip_index = PipIndex::new(&host);
        let nb_index = NearestBoundaryIndex::new(seg_index.clone());
        Ok(Self {
            host,
            seg_index,
            pip_index,
            nb_index,
            candidate_buf: Vec::new(),
        })
    }
```

</details>



##### `clear_candidates` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">pub(crate)</span>


```rust
fn clear_candidates (& mut self)
```

<details>
<summary>Source</summary>

```rust
    pub(crate) fn clear_candidates(&mut self) {
        self.candidate_buf.clear();
    }
```

</details>





