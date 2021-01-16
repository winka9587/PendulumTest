
import * as THREE from 'PendulumTest/js/three.js-master/build/three.module.js';



class Joint {//关节类
    constructor() {
        this.position = new THREE.Vector3(0,0,0);
        this.prevPosition = new THREE.Vector3(0,0,0);
        this.speed = new THREE.Vector3(0,0,0);
        this.acceleration = new THREE.Vector3(0,0,0);
        this.reset();
    }
    calc() {
        let prevSpeed = this.speed.clone();//记录上一帧速度
        this.speed.subVectors( this.position , this.prevPosition);//当前位置减上一帧位置得到当前速度
        this.prevPosition.copy( this.position );//当前位置保存为上一帧,供下次计算速度使用
        this.acceleration.subVectors( this.speed,prevSpeed);//加速度等于速度相减
    }
    reset() {
        this.prevPosition.copy(this.position);
        this.speed.setX(0,0,0);
        this.speed.setY(0,0,0);
        this.speed.setZ(0,0,0);//当前速度设置为0
        this.acceleration.setX(0);
        this.acceleration.setY(0);
        this.acceleration.setZ(0);//当前加速度设置为0
    }
}


class Segment {
    constructor() {
        //一段包含两个关节head(head)和tail(tail)
        this.head = new Joint();
        this.tail = new Joint();

        //每个segment都有其θ和φ
        this.theta = 0.1;
        this.phi = 0.0;
        this.dTheta = 0.0;//θ和φ对t的导数
        this.dPhi = 0.0;
        this.l = 0.52;//segment长度
        this.c_a = 0.0;//角摩擦
        this.c_d = 0.0;//摩擦阻力
        this.mass = 1.0;//质量
        this.force = new THREE.Vector3(0, 0, -5);
        this.air = new THREE.Vector3(0, 0, 0);
        
        //求解器
        this.solver = new window.odex_solver(4);
        //this.solver.absoluteTolerance = this.solver.relativeTolerance = 1e-10;
        let pendulum = this;
        // SOLVER
        this.equation = function(x, y) {
            // console.log("input t="+x+"  ("
            //                     + y[0]+" , "
            //                     + y[1]+" , "
            //                     + y[2]+" , "
            //                     + y[3]+" ) "
            //                     );
            let [θ, dθ, φ, dφ] = y;
            let [l, m] = [pendulum.l, pendulum.mass];
            let [c_a, c_d] = [pendulum.c_a, pendulum.c_d];
            let [gx, gy, gz] = [pendulum.force.x, pendulum.force.y, pendulum.force.z];
            let [wx, wy, wz] = [pendulum.air.x, pendulum.air.y, pendulum.air.z];//计算相对速度

            // [keep this for wolframalpha]
            // d/dt ((-l sin(θ) sin(φ) p + l cos(θ) cos(φ) t) - x)^2 + ((l sin(θ) cos(φ) p + l cos(θ) sin(φ) t) - y)^2 + (l sin(θ)t - z)^2
            let vw2 = Math.pow(-l *Math.sin(θ) *Math.sin(φ) *dφ + l *Math.cos(θ) *Math.cos(φ) *dθ - wx, 2)
                    + Math.pow(l *Math.sin(θ) *Math.cos(φ) *dφ + l *Math.cos(θ) *Math.sin(φ) *dθ - wy, 2)
                    + Math.pow(l *Math.sin(θ) *dθ - wz, 2);
            
            let vw121 = Math.pow(vw2, 1/2);
            

            let c_θ = 0;
            let c_φ = 0;
            // Angular Friction [c_a]
            c_θ += c_a * l * l * dθ;
            c_φ += c_a * l * l * dφ * Math.pow(Math.sin(θ), 2);
            // Drag Friction    [c_d]
            c_θ += c_d  * vw121 * (
                  2 *l *Math.cos(θ) * Math.cos(φ) *(l * dθ * Math.cos(θ) * Math.cos(φ) -l * dφ * Math.sin(θ) * Math.sin(φ) - wx)
                + 2 *l *Math.cos(θ) * Math.sin(φ)* (l* dθ* Math.cos(θ) *Math.sin(φ) +l *dφ *Math.sin(θ) *Math.cos(φ) - wy)
                + 2 *l *Math.sin(θ) *(l *dθ *Math.sin(θ) - wz)
                );
            c_φ += c_d  * vw121 * (
                2 *l *Math.sin(θ) *(l* dφ* Math.sin(θ)* (Math.pow(Math.sin(φ), 2)
                + Math.pow(Math.cos(φ), 2))
                + wx*Math.sin(φ) - wy*Math.cos(φ))
                );

            let d_θ = dθ;
            let d_dθ =  Math.pow(dφ, 2) / 2 * Math.sin(2*θ)
                        + Math.sin(θ)/l * gz
                        + Math.cos(θ)/l * (gx * Math.cos(φ) + gy * Math.sin(φ))
                        - c_θ  / l / l;



            let d_φ = dφ;
            let d_dφ =  (-2) * dθ * dφ / Math.tan(θ)
                        + 1/(Math.sin(θ)*l) * (-gx*Math.sin(φ) + gy*Math.cos(φ))
                        - c_φ  / l / l / Math.pow(Math.sin(θ), 2);
            // console.log("output t="+x+"  ("
            //                     + d_θ + " , "
            //                     + d_dθ + " , "
            //                     + d_φ + " , "
            //                     + d_dφ + " ) "
            //                     );
            return [d_θ, d_dθ, d_φ, d_dφ];
        };
    }

    step(time_step) {
        console.log("====step====");
        // Calculate force, resulting from movement.
        let result = this.solver.solve(this.equation,//等式
                                0, //
                                [this.theta, this.dTheta, this.phi, this.dPhi], //
                                time_step);//
        if (Number.isNaN(result.y[0]))
            return;
        [this.theta, this.dTheta, this.phi, this.dPhi] =  result.y;
        console.log("result = ("
                                + result.y[0] + " , "
                                + result.y[1] + " , "
                                + result.y[2] + " , "
                                + result.y[3] + " ) "
                                );
    }

    updatePosition() {
        let pos = new THREE.Vector3(
            this.l * Math.sin(this.theta) * Math.cos(this.phi),
            this.l * Math.sin(this.theta) * Math.sin(this.phi),
            -this.l * Math.cos(this.theta)
            );
        pos.add(this.head.position);
        this.tail.position.copy(pos);
    }

    

    // draw(ctx) {
    //     // Visualization
    //     let point = new Point(0, 0, 0);
    //     point.size = 5;
    //     point.position = this.tail.position;
    //     point.draw(ctx);

    //     let line = new Line();
    //     line.start = this.head.position; line.end = this.tail.position;
    //     line.draw(ctx);
        
    //     let line_force = new Line();
    //     line_force.color = "rgba(119, 200, 255, 0.8)";
    //     line_force.start = this.head.position;
    //     line_force.end = this.force.multiply(0.01).add(this.head.position);
    //     line_force.draw(ctx);
    // }
}

//如果只有一段可以直接把Segment类作为Pendulum类使用
//单摆类
export class Pendulum {
    constructor(param) {
        console.log("param");
        console.log(param);
        this.head = new THREE.Vector3(0,0,0);

        this.c_a = param.c_a;//0.5
        this.c_d = param.c_d;//2

        this.gravity = new THREE.Vector3(0, 0, -10);
        this.wind = param.wind.clone();
        this.time_step = param.time_step;
        
        this.segment = new Segment();
        this.segment.c_a = this.c_a;
        this.segment.c_d = this.c_d;
        this.segment.head.position = this.head;
        this.segment.theta = param.theta;//设置初始位置
        this.segment.phi = param.phi;
        this.segment.updatePosition();
        this.segment.tail.reset();
    }

    step() {
        this.segment.head.calc();
        this.segment.tail.calc();

        let childForce = new THREE.Vector3();
        this.segment.force.copy(this.gravity);
        this.segment.force.add(this.segment.head.acceleration.divideScalar(-this.time_step*this.time_step).multiplyScalar(this.segment.mass));
        this.segment.air.copy(this.wind);
        this.segment.air.add(this.segment.head.speed.multiplyScalar(-this.time_step));
        this.segment.step(this.time_step);
        this.updatePosition();
    }

    updatePosition() {
        
        this.segment.updatePosition();
        
    }
    updatefriction(){
        this.segment.c_a = this.c_a;
        this.segment.c_d = this.c_d;
        
    }
    changespeed(value){
        console.log(this.segment.tail);
        console.log(value);
        this.segment.tail.reset();
        this.segment.tail.speed.copy(value);
        console.log(this.segment.tail);
    }
}
