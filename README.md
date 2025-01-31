# Catenary Transit Routing Engine

Implementation of Transfer Patterns[^1] and Scalable Transfer Patterns 

Supplemented by exercises from Lectures 9-11 from Uni Freiburg's [Efficient Route Planning course (2012 Summer)](https://ad-wiki.informatik.uni-freiburg.de/teaching/EfficientRoutePlanningSS2012) with Professor Hannah Bast

[1^] Bast, H. et al. (2010). Fast Routing in Very Large Public Transportation Networks Using Transfer Patterns. In: de Berg, M., Meyer, U. (eds) Algorithms – ESA 2010. ESA 2010. Lecture Notes in Computer Science, vol 6346. Springer, Berlin, Heidelberg. https://doi.org/10.1007/978-3-642-15775-2_25

[2^]: H. Bast, M. Hertel, and S. Storandt, “Scalable Transfer Patterns,” 2016 Proceedings of the Meeting on Algorithm Engineering and Experiments (ALENEX), pp. 15–29, Jan. 2016, doi: https://doi.org/10.1137/1.9781611974317.2.

‌
### Running the demo for Montreal

Extracting from a compressed bincode is faster.

Run this to produce a bincode from the catenary-backend.

```bash
cargo run --release --bin osm_extractor -- --routing_export_path ./testing_routing_export --temp_dir ./testing_temp_dir
```

Download le file GTFS de STM 
```bash
wget https://www.stm.info/sites/default/files/gtfs/gtfs_stm.zip
```

### Query Graph flag

To produce a query graph, for example, on the first run, execute
`./target/release/transit-router --makequerygraph true`

or `cargo run --release -- --makequerygraph true`

to reuse, put false
