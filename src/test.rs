use regex::Regex;
use std::fs::*;

fn main() {
    let dirpath = "test_unzip";
    let re = Regex::new(r"(.+)\.").unwrap();
    let entries = read_dir(dirpath)
        .unwrap()
        .map(|res| res.map(|e| e.path()))
        .collect::<Vec<_>>();

    for entry in entries {
        let file = entry.unwrap();
        let path = file.as_path().to_str().unwrap();
        let filename = re.captures(path).unwrap().extract::<1>().0;
        let savepath = format!("{dirpath}\\{filename}.json");

        File::create(savepath.clone()).unwrap();
    }
}