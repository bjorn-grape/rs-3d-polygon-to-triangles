use std::fs::File;
use std::io::Write;
use rs_3d_polygon_to_triangles::triangularize_polygon;


fn print_triangles_in_python_format(p: &Vec<(f32, f32, f32)>, file: &mut File) {
    file.write(b"#! /bin/env/python3\n");
    file.write(b"#/!\\ Generated file => run me with python3\n");
    file.write(b"from base import make_plot\n");
    let res = triangularize_polygon(p);
    file.write(b"list_index = [");
    let mut index: usize = 0;
    let tri = match res {
        Some(elm) => elm,
        None => { assert!(false); vec![] }
    };
    let maxlen = &tri.len();
    for elm in tri {
        file.write(format!("[{},{},{}]", elm[0], elm[1], elm[2]).as_ref());
        if index != maxlen - 1 {
            file.write(b",");
        }
        index += 1;
    }
    file.write(b"]\n");
    file.write(b"list_points = [");
    let maxlen2 = &p.len();
    for index2 in 0..(p.len()) {
        file.write(format!("[{},{},{}]", &p[index2].0, &p[index2].1, &p[index2].2).as_ref());
        if index2 != maxlen2 - 1 {
            file.write(b",");
        }
    }

    file.write(b"]\n");
    file.write(b"make_plot(list_index, list_points)\n");
}


#[test]
fn print_square_to_triangles() {
    let mut file_out = File::create("python_testing/test1.py").unwrap();
    let p: Vec<(f32, f32, f32)> =
        vec![
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, 1.0, 1.0),
            (0.0, 1.0, 0.0)
        ];

    print_triangles_in_python_format(&p, &mut file_out);
}

/// You can view the related polygon here
/// https://www.geogebra.org/3d/jzytfskk
#[test]
fn print_concave_18tagon() {
    let mut file_out = File::create("python_testing/test3.py").unwrap();

    let p =
        vec![
            (-5.000000000000, 2.000000000000, 0.000000000000), // A
            (-2.000000000000, -0.5000000000000, 2.000000000000), // B
            (-3.437830000000, -3.762710000000, 0.000000000000), // C
            (0.2878375872107, -6.011129618733, 2.683691852387), // D
            (-0.7172545777264, -2.613644090124, 2.611269318740), // E
            (5.123746231974, -4.321487522197, 7.242926670790), // F
            (1.230198288741, -1.856134371178, 4.465298839044), // G
            (5.191134065668, -0.09767940749428, 8.287056817222), // H
            (5.207033647170, 1.226119544453, 8.609805272946), // I
            (4.790806317938, 2.523806087794, 8.554302233011), // J
            (4.338665579674, 3.791845117776, 8.460948397246), // K
            (2.226703985533, 2.992991838246, 6.455588968661), // L
            (1.480359674091, 1.609352493100, 5.489798315307), // M
            (0.9381937692857, 0.6894647883320, 4.808117615174), // N
            (1.573233371040, 5.901431367591, 6.571815895088), // O
            (-0.1686168235859, 6.659749358895, 5.248741685711), // P
            (0.9129904320214, -2.698786229022, 3.995387510839), // Q
            (-4.563954906766, 4.668390903673, 0.9984949436191) // R
        ];
    print_triangles_in_python_format(&p, &mut file_out);
}