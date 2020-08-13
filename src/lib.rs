use nalgebra::{Vector3, Vector2};
use std::f32::EPSILON;
use std::collections::LinkedList;
use std::borrow::BorrowMut;

fn is_coplanar(polygon: &Vec<usize>, positions: &Vec<Vector3<f32>>) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    if polygon.len() == 3 {
        return true;
    }
    let large_epsilon = EPSILON * 100.0; // sometimes the precision of mesh is low
    let mut plan_normals: Vec<Vector3<f32>> = vec![];
    let vec1: Vector3<f32> = &positions[polygon[1] as usize] - &positions[polygon[0] as usize];
    for i in 0..(polygon.len() - 2) {
        let vec2: Vector3<f32> = &positions[polygon[i + 2] as usize] - &positions[polygon[0] as usize];
        plan_normals.push(vec1.cross(&vec2).normalize());
    }
    let n1 = &plan_normals[0];
    for i in 0..(plan_normals.len() - 1) {
        let n2 = &plan_normals[i + 1];
        let norm = (n1 - n2).norm(); // same direction vector
        let norm_inv = (n1 + n2).norm(); // opposite direction vector
        if norm > large_epsilon && norm_inv > large_epsilon
        {
            return false;
        }
    }
    return true;
}

fn project_to_2d(polygon: &Vec<usize>, positions: &Vec<Vector3<f32>>) -> Vec<Vector2<f32>> {
    let e1: Vector3<f32> = &positions[polygon[1] as usize] - &positions[polygon[0] as usize];
    let tmp: Vector3<f32> = &positions[polygon[2] as usize] - &positions[polygon[0] as usize];
    let crossed = e1.cross(&tmp).normalize();
    let e2 = -e1.cross(&crossed);
    let v0: &Vector3<f32> = &positions[polygon[0] as usize];
    let x0 = v0[0];
    let y0 = v0[1];
    let z0 = v0[2];
    let e1x = e1[0];
    let e1y = e1[1];
    let e1z = e1[2];
    let e2x = e2[0];
    let e2y = e2[1];
    let e2z = e2[2];
    let mut projected_2d_points: Vec<Vector2<f32>> = vec![];
    for elm in polygon {
        let v1: &Vector3<f32> = &positions[*elm as usize];
        let x1 = v1[0];
        let y1 = v1[1];
        let z1 = v1[2];
        let x_f: f32 = (x1 - x0) * e1x + (y1 - y0) * e1y + (z1 - z0) * e1z;
        let y_f: f32 = (x1 - x0) * e2x + (y1 - y0) * e2y + (z1 - z0) * e2z;
        projected_2d_points.push(Vector2::new(x_f, y_f))
    }
    projected_2d_points
}


pub fn cross_2d(v1: &Vector2<f32>, v2: &Vector2<f32>) -> f32 {
    return &v1[0] * &v2[1] - &v1[1] * &v2[0];
}

pub fn triangularize_polygon(v: &Vec<(f32, f32, f32)>) -> Option<Vec<Vector3<usize>>>
{
        let all_pts: Vec<Vector3<f32>> = v.iter()
            .map(|&x| Vector3::new(x.0, x.1, x.2)).collect();
        let numbers = 0..;
        let polygon_indexes: Vec<usize> = numbers.take(v.len()).borrow_mut().collect();
        return triangularize_index_list(&polygon_indexes, &all_pts);
}

// using the ear clipping method
pub fn triangularize_index_list(index_list: &Vec<usize>, point3d_list: &Vec<Vector3<f32>>) -> Option<Vec<Vector3<usize>>> {
    if index_list.len() < 3 || !is_coplanar(index_list, point3d_list) {
        return None;
    }
    let polygon_2d = project_to_2d(index_list, point3d_list);

    let mut triangle_list_2d: Vec<Vector3<usize>> = vec![];



    let mut index_linked_list: LinkedList<usize> = LinkedList::new();
    for pass in 1..4 {
        index_linked_list = LinkedList::new();
        match pass {
            1 => {
                for i in (0 as usize)..polygon_2d.len() {
                    &index_linked_list.push_back(i);
                }
            }
            2 => {
                for i in (0 as usize)..polygon_2d.len() {
                    &index_linked_list.push_back(polygon_2d.len() - 1 - i);
                }
            }
            _ => { return None; }
        }
        let max_len = index_linked_list.len();
        let maxmax_len = index_linked_list.len();
        let mut turn: usize = 0;
        while &index_linked_list.len() > &(3 as usize) {
            turn += 1;
            if turn > max_len && max_len == index_linked_list.len() || turn > maxmax_len * maxmax_len {
                println!("TOO LOONG.... SKIP");
                break;
            }
            let i0 = index_linked_list.pop_front().unwrap();
            let i1 = index_linked_list.pop_front().unwrap();
            let i2 = index_linked_list.pop_front().unwrap();
            let x0 = &polygon_2d[i0];
            let x1 = &polygon_2d[i1];
            let x2 = &polygon_2d[i2];
            let v1 = x1 - x0;
            let v2 = x2 - x0;
            let cross_v1_v2 = cross_2d(&v1, &v2);
            if cross_v1_v2 < 0.0 {
                index_linked_list.push_front(i2);
                index_linked_list.push_front(i1);
                index_linked_list.push_back(i0);
                continue;
            }
            let u0 = x1 - x0;
            let u1 = x2 - x1;
            let u2 = x0 - x2;

            let mut pt_in_triangle_exist = false;
            for &elm in &index_linked_list {
                let pt = &polygon_2d[elm];
                let w0 = pt - x0;
                let cross0 = cross_2d(&u0, &w0);
                let w1 = pt - x1;
                let cross1 = cross_2d(&u1, &w1);
                let w2 = pt - x2;
                let cross2 = cross_2d(&u2, &w2);
                if cross0 > 0.0 && cross1 > 0.0 && cross2 > 0.0 {
                    // point in triangle cannot make mesh
                    pt_in_triangle_exist = true;
                    break;
                }
            }
            if pt_in_triangle_exist {
                index_linked_list.push_front(i2);
                index_linked_list.push_front(i1);
                index_linked_list.push_back(i0);
                continue;
            }
            println!("=======> GOOD");
            triangle_list_2d.push(Vector3::new(i0, i1, i2));
            index_linked_list.push_front(i2);
            index_linked_list.push_back(i0);
        }
        if index_linked_list.len() <= 3 {
            break;
        }
    }
    let i0 = index_linked_list.pop_front().unwrap();
    let i1 = index_linked_list.pop_front().unwrap();
    let i2 = index_linked_list.pop_front().unwrap();
    triangle_list_2d.push(Vector3::new(i0, i1, i2));
    let triangle_list = triangle_list_2d.iter()
        .map(|elm| Vector3::new(index_list[elm[0]], index_list[elm[1]], index_list[elm[2]]))
        .collect();
    return Some(triangle_list);
}


#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use std::borrow::BorrowMut;

    fn polygon_builder(v: Vec<(f32, f32, f32)>) -> (Vec<usize>, Vec<Vector3<f32>>)
    {
        let all_pts: Vec<Vector3<f32>> = v.iter()
            .map(|&x| Vector3::new(x.0, x.1, x.2)).collect();
        let numbers = 0..;
        let polygon_indexes: Vec<usize> = numbers.take(v.len()).borrow_mut().collect();
        return (polygon_indexes, all_pts);
    }

    fn test_coplanar(p: (Vec<usize>, Vec<Vector3<f32>>), expectation: bool) {
        let res = is_coplanar(&p.0, &p.1);
        assert_eq!(res, expectation);
    }

    #[test]
    fn coplanar_square() {
        let p = polygon_builder(
            vec![
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
                (0.0, 1.0, 1.0),
                (0.0, 1.0, 0.0)
            ]);
        let expectation = true;

        test_coplanar(p, expectation);
    }


    #[test]
    fn not_coplanar_square() {
        let p = polygon_builder(
            vec![
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
                (0.0, 1.0, 1.0),
                (0.0, 1.0, 0.0)
            ]);
        let expectation = false;
        test_coplanar(p, expectation);
    }

    /// You can view the related polygon here
    /// https://www.geogebra.org/3d/qr4vgg33
    #[test]
    fn coplanar_convex_pentagon() {
        let p = polygon_builder(
            vec![
                (0.0, 0.0, 4.0),
                (4.0, 0.0, 4.0),
                (6.0, 2.5, 2.0),
                (4.0, 5.0, 0.0),
                (0.0, 5.0, 0.0)
            ]);
        let expectation = true;
        test_coplanar(p, expectation);
    }

    /// You can view the related polygon here
    /// https://www.geogebra.org/3d/y4wbnm59
    #[test]
    fn coplanar_concave_pentagon() {
        let p = polygon_builder(
            vec![
                (0.0, 0.0, 4.0),
                (4.0, 0.0, 4.0),
                (2.0, 2.5, 2.0),
                (4.0, 5.0, 0.0),
                (0.0, 5.0, 0.0)
            ]);
        let expectation = true;
        test_coplanar(p, expectation);
    }

    /// You can view the related polygon here
    ///
    #[test]
    fn coplanar_concave_18tagon() {
        let p = polygon_builder(
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
            ]);
        let expectation = true;
        test_coplanar(p, expectation);
    }

    /// You can view the related polygon here
    /// https://www.geogebra.org/3d/xjs5mqdc
    #[test]
    fn non_coplanar_polygon() {
        let p = polygon_builder(
            vec![
                (0.0, 0.0, 4.0),
                (4.0, 0.0, 4.0),
                (2.0, 2.5, 3.0),
                (4.0, 5.0, 0.0),
                (0.0, 5.0, 0.0)
            ]);
        let expectation = false;
        test_coplanar(p, expectation);
    }


    fn test_triangularized(p: (Vec<usize>, Vec<Vector3<f32>>), size: usize) {
        let res = triangularize_index_list(&p.0, &p.1);
        assert_ne!(res, None);
        assert_eq!(res.unwrap().len(), size);
    }

    #[test]
    fn square_to_triangles() {
        let p = polygon_builder(
            vec![
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
                (0.0, 1.0, 1.0),
                (0.0, 1.0, 0.0)
            ]);
        test_triangularized(p, 2);
    }
}