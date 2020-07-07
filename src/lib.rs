use nalgebra::{Vector3, Vector2};
use std::f32::EPSILON;
use std::collections::LinkedList;

fn is_coplanar(polygon: &Vec<usize>, positions: &Vec<Vector3<f32>>) -> bool {
    let mut plan_normals: Vec<Vector3<f32>> = vec![];
    let vec1: Vector3<f32> = &positions[polygon[1] as usize] - &positions[polygon[0] as usize];
    let s0 = format!("{:?}", positions);
    let s1 = format!("{:?}", vec1);
    for i in 0..(polygon.len() - 2) {
        let vec2: Vector3<f32> = &positions[polygon[i + 2] as usize] - &positions[polygon[0] as usize];
        let s2 = format!("{:?}", vec2);
        plan_normals.push(vec1.cross(&vec2).normalize());
        let s3 = format!("{:?}", plan_normals);
    }
    let n1 = &plan_normals[0];
    let s4 = format!("{:?}", n1);
    for i in 0..(plan_normals.len() - 1) {
        let n2 = &plan_normals[i + 1];
        let s5 = format!("{:?}", n2);
        let norm = (n1 - n2).norm();
        let s6 = format!("{:?}", norm);
        if norm > EPSILON
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


pub fn triangularize(polygon: &Vec<usize>, positions: &Vec<Vector3<f32>>) -> Option<Vec<Vector3<usize>>> {
    if polygon.len() < 3 || !is_coplanar(polygon, positions) {
        return None;
    }
    let polygon_2d = project_to_2d(polygon, positions);

    let mut triangle_list_2d: Vec<Vector3<usize>> = vec![];

    let mut index_linked_list: LinkedList<usize> = LinkedList::new();
    for i in (0 as usize)..polygon_2d.len() {
        &index_linked_list.push_back(i);
    }
    let max_len = index_linked_list.len();
    while &index_linked_list.len() > &(3 as usize) {
        let i0 = index_linked_list.pop_front().unwrap();
        let i1 = index_linked_list.pop_front().unwrap();
        let i2 = index_linked_list.pop_front().unwrap();
        let x0 = &polygon_2d[i0];
        let x1 = &polygon_2d[i1];
        let x2 = &polygon_2d[i2];
        let v1 = x1 - x0;
        let v2 = x2 - x0;
        if cross_2d(&v1, &v2) < 0.0 {
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
            let dot0 = u0.dot(&w0);
            let w1 = pt - x1;
            let dot1 = u1.dot(&w1);
            let w2 = pt - x2;
            let dot2 = u2.dot(&w2);
            if dot0 > 0.0 && dot1 > 0.0 && dot2 > 0.0 {
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
        triangle_list_2d.push(Vector3::new(i0, i1, i2));
        index_linked_list.push_front(i2);
        index_linked_list.push_back(i0);
    }
    let i0 = index_linked_list.pop_front().unwrap();
    let i1 = index_linked_list.pop_front().unwrap();
    let i2 = index_linked_list.pop_front().unwrap();
    triangle_list_2d.push(Vector3::new(i0, i1, i2));
    let triangle_list = triangle_list_2d.iter()
        .map(|elm| Vector3::new(polygon[elm[0]], polygon[elm[1]], polygon[elm[2]]))
        .collect();
    return Some(triangle_list);
}


#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }

    #[test]
    fn coplanar_square() {
        let p1 = Vector3::new(0.0, 0.0, 0.0);
        let p2 = Vector3::new(0.0, 0.0, 1.0);
        let p3 = Vector3::new(0.0, 1.0, 1.0);
        let p4 = Vector3::new(0.0, 1.0, 0.0);
        let all_pts: Vec<Vector3<f32>> = vec![p1, p2, p3, p4];
        let polygon_indexes: Vec<usize> = vec![0, 1, 2, 3];
        let res = is_coplanar(&polygon_indexes, &all_pts);
        assert_eq!(res, true);
    }

    #[test]
    fn not_coplanar_square() {
        let p1 = Vector3::new(1.0, 0.0, 0.0);
        let p2 = Vector3::new(0.0, 0.0, 1.0);
        let p3 = Vector3::new(0.0, 1.0, 1.0);
        let p4 = Vector3::new(0.0, 1.0, 0.0);
        let all_pts: Vec<Vector3<f32>> = vec![p1, p2, p3, p4];
        let polygon_indexes: Vec<usize> = vec![0, 1, 2, 3];
        let res = is_coplanar(&polygon_indexes, &all_pts);
        assert_eq!(res, false);
    }
}