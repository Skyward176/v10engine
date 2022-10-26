import "engine_sim.mr"

units units()
constants constants()
impulse_response_library ir_lib()

private node wires {
    output wire1: ignition_wire();
    output wire2: ignition_wire();
    output wire3: ignition_wire();
    output wire4: ignition_wire();
    output wire5: ignition_wire();
    output wire6: ignition_wire();
    output wire7: ignition_wire();
    output wire8: ignition_wire();
    output wire9: ignition_wire();
    output wire10: ignition_wire();
}

label cycle(2* 360 * units.deg)
public node vemsign {
    input wires;
    input timing_curve;
    input rev_limit: 20000 * units.rpm;
    alias output __out:
        ignition_module(timing_curve: timing_curve, rev_limit: rev_limit)
            .connect_wire(wires.wire1, (0.0/10.0)*cycle)
            .connect_wire(wires.wire2, (1.0/10.0)*cycle)
            .connect_wire(wires.wire3, (2.0/10.0)*cycle)
            .connect_wire(wires.wire4, (3.0/10.0)*cycle)
            .connect_wire(wires.wire7, (4.0/10.0)*cycle)
            .connect_wire(wires.wire8, (5.0/10.0)*cycle)
            .connect_wire(wires.wire9, (6.0/10.0)*cycle)
            .connect_wire(wires.wire10, (7.0/10.0)*cycle)
            .connect_wire(wires.wire5, (8.0/10.0)*cycle)
            .connect_wire(wires.wire6, (9.0/10.0)*cycle)
}
public node v10_camshaft_builder {
    input intake_lobe_profile;
    input exhaust_lobe_profile;
    input lobe_separation: 114.0 * units.deg;
    input intake_lobe_center: lobe_separation;
    input exhaust_lobe_center: lobe_separation;
    input advance: 0.0 * units.deg;
    input base_radius: 0.5 * units.inch;

    output intake_cam0: _intake_cam0;
    output exhaust_cam0: _exhaust_cam0;
    output intake_cam1: _intake_cam1;
    output exhaust_cam1: _exhaust_cam1;

    camshaft_parameters params(
        advance: advance,
        base_radius: base_radius
    )

    camshaft _intake_cam0(params, lobe_profile: intake_lobe_profile)
    camshaft _exhaust_cam0(params, lobe_profile: exhaust_lobe_profile)
    camshaft _intake_cam1(params, lobe_profile: intake_lobe_profile)
    camshaft _exhaust_cam1(params, lobe_profile: exhaust_lobe_profile)

    label rot(72 * units.deg)
    label rot360(360 * units.deg)

    //firing order
    // 1 2 3 4 7 8 9 10 5 6
    _exhaust_cam0
        .add_lobe(rot360 - exhaust_lobe_center)
        .add_lobe((rot360 - exhaust_lobe_center) + 2 * rot)
        .add_lobe((rot360 - exhaust_lobe_center) + 8 * rot)
        .add_lobe((rot360 - exhaust_lobe_center) + 4 * rot)
        .add_lobe((rot360 - exhaust_lobe_center) + 6 * rot)

    _intake_cam0
        .add_lobe(rot360 + intake_lobe_center)
        .add_lobe(rot360 + intake_lobe_center + 2 * rot)
        .add_lobe(rot360 + intake_lobe_center + 8 * rot)
        .add_lobe(rot360 + intake_lobe_center + 4 * rot)
        .add_lobe(rot360 + intake_lobe_center + 6 * rot)
    _exhaust_cam1
        .add_lobe(rot360 - exhaust_lobe_center + 1 * rot)
        .add_lobe((rot360 - exhaust_lobe_center) + 3 * rot)
        .add_lobe((rot360 - exhaust_lobe_center) + 9 * rot)
        .add_lobe((rot360 - exhaust_lobe_center) + 5 * rot)
        .add_lobe((rot360 - exhaust_lobe_center) + 7 * rot)

    _intake_cam1
        .add_lobe(rot360 + intake_lobe_center + 1 * rot)
        .add_lobe(rot360 + intake_lobe_center + 3 * rot)
        .add_lobe(rot360 + intake_lobe_center + 9 * rot)
        .add_lobe(rot360 + intake_lobe_center + 5 * rot)
        .add_lobe(rot360 + intake_lobe_center + 7 * rot)
}
public node v10 {
    alias output __out: engine;

    wires wires()

    engine engine(
        name: "v10",
        starter_torque: 200* units.lb_ft,
        redline: 20000*units.rpm,
        fuel: fuel(
            max_turbulence_effect:4.0,
            burning_efficiency_randomness: 0.2,
            max_burning_efficiency: 0.85)
    )
    
    crankshaft c0(
        throw: 0.5 * 79.0 * units.mm,
        flywheel_mass: 20 * units.lb,
        mass: 40* units.lb,
        friction_torque: 10.0 * units.lb_ft,
        moment_of_intertia: 0.22986844776863666 * 0.9,
        position_x: 0.0,
        position_y: 0.0,
        tdc: 72*units.deg
    )
    rod_journal_rj0(angle: 0*72*units.deg)
    rod_journal_rj1(angle: 2*72* units.deg)
    rod_journal_rj2(angle: 3*72* units.deg)
    rod_journal_rj3(angle: 4*72* units.deg)
    rod_journal_rj4(angle: 1*72* units.deg)

    c0
        .add_rod_journal(rj0)
        .add_rod_journal(rj1)
        .add_rod_journal(rj2)
        .add_rod_journal(rj3)
        .add_rod_journal(rj4)

    engine.add_crankshaft(c0)

    cylinder_bank_parameters bank_params(
        bore: 88 * units.mm 
        deck_height: (79* units.mm) / 2 + (130 * units.mm) + (1.0*units.inch)
    )

    connecting_rod_parameters cr_params(
        mass:50*units.g,
        moment_of_intertia: rod_moment_of_intertia(
            mass:50*units.g,
            length:130*units.mm,
        ),
        center_of_mas: 0.0,
        length: 130*units.mm

    )
    cylinder_bank b0(bank_params, angle:-36);
    cylinder_bank b1(bank_params, angle:36);
   
    intake intake(
        plenum_volume: 1.325 * units.L,
        plenum_cross_section_area: 20.0 * units.cm2,
        intake_flow_rate: k_carb(1000.0),
        runner_flow_rate: k_carb(200.0),
        runner_length: 4.0 * units.inch,
        idle_flow_rate: k_carb(0.0),
        idle_throttle_plate_position: 0.998,
        velocity_decay: 0.5
    )

    exhaust_system_parameters es_params(
        outlet_flow_rate: k_carb(2000.0),
        primary_tube_length: 50.0 * units.inch,
        primary_flow_rate: k_carb(1000.0),
        velocity_decay: 1.0
    )

    exhaust_system exhaust0(
        es_params,
        audio_volume: 2.0,
        length: 100 * units.inch,
        impulse_response: ir_lib.mild_exhaust_0_reverb
    )
    exhaust_system exhaust1(
        es_params,
        audio_volume: 2.0,
        length: 100.5 * units.inch,
        impulse_response: ir_lib.mild_exhaust_0_reverb
    )
    piston_parameters piston_params(
        mass: (100) * units.g, // 414 - piston mass, 152 - pin weight
        compression_height: 1.0 * units.in,
        wrist_pin_position: 0.0,
        displacement: 0.0
    )
    b0
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.0)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj0,
            intake: intake,
            exhaust_system: exhaust1,
            ignition_wire: wires.wire1,
            sound_attenuation: 0.8,
            primary_length: 5 * units.cm
        )
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.0)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj1,
            intake: intake,
            exhaust_system: exhaust1,
            ignition_wire: wires.wire3,
            sound_attenuation: 1.0,
            primary_length: 4 * units.cm
        )
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.0)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj2,
            intake: intake,
            exhaust_system: exhaust1,
            ignition_wire: wires.wire5,
            sound_attenuation: 1.1,
            primary_length: 3 * units.cm
        )
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.0)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj3,
            intake: intake,
            exhaust_system: exhaust1,
            ignition_wire: wires.wire7,
            sound_attenuation: 0.9,
            primary_length: 2 * units.cm
        )
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.0)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj4,
            intake: intake,
            exhaust_system: exhaust1,
            ignition_wire: wires.wire9,
            sound_attenuation: 0.7,
            primary_length: 1 * units.cm
        )
        .set_cylinder_head(
            v10_72_head(
                intake_camshaft: camshaft.intake_cam_0,
                exhaust_camshaft: camshaft.exhaust_cam_0,
                flip_display: true,
                flow_attenuation: 1.5)
        )
    b1
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.0)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj0,
            intake: intake,
            exhaust_system: exhaust0,
            ignition_wire: wires.wire2,
            sound_attenuation: 0.7,
            primary_length: 5 * units.cm
        )
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.0)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj1,
            intake: intake,
            exhaust_system: exhaust0,
            ignition_wire: wires.wire4,
            sound_attenuation: 0.8,
            primary_length: 4 * units.cm
        )
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.0)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj2,
            intake: intake,
            exhaust_system: exhaust0,
            ignition_wire: wires.wire6,
            primary_length: 3 * units.cm
        )
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.0)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj3,
            intake: intake,
            exhaust_system: exhaust0,
            ignition_wire: wires.wire8,
            sound_attenuation: 1.1,
            primary_length: 2 * units.cm
        )
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.0)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj4,
            intake: intake,
            exhaust_system: exhaust0,
            ignition_wire: wires.wire10,
            sound_attenuation: 0.7,
            primary_length: 1 * units.cm
        )
        .set_cylinder_head(
            v10_72_head(
                intake_camshaft: camshaft.intake_cam_1,
                exhaust_camshaft: camshaft.exhaust_cam_1,
                flow_attenuation: 1.5)
        )

    engine
        .add_cylinder_bank(b0)
        .add_cylinder_bank(b1)

    v10_camshaft_builder camshaft()
    
    function timing_curve(1000*units.rpm)
    timing_curve
        .add_sample(0000 * units.rpm, 12*units.deg)
        .add_sample(1000 * units.rpm, 12*units.deg)
        .add_sample(2000 * units.rpm, 20*units.deg)
        .add_sample(3000 * units.rpm, 26*units.deg)
        .add_sample(4000 * units.rpm, 30*units.deg)
        .add_sample(5000 * units.rpm, 34*units.deg)
        .add_sample(6000 * units.rpm, 38*units.deg)
        .add_sample(7000 * units.rpm, 38*units.deg)
        .add_sample(8000 * units.rpm, 38*units.deg)
        .add_sample(9000 * units.rpm, 38*units.deg)
        .add_sample(10000 * units.rpm, 38*units.deg)
        .add_sample(11000 * units.rpm, 38*units.deg)
        .add_sample(12000 * units.rpm, 38*units.deg)
        .add_sample(13000 * units.rpm, 38*units.deg)
        .add_sample(14000 * units.rpm, 38*units.deg)
        .add_sample(15000 * units.rpm, 38*units.deg)
        .add_sample(16000 * units.rpm, 38*units.deg)
        .add_sample(17000 * units.rpm, 38*units.deg)
        .add_sample(18000 * units.rpm, 38*units.deg)
        .add_sample(19000 * units.rpm, 38*units.deg)
        .add_sample(20000 * units.rpm, 38*units.deg)
    
    engine.add_ignition_module(
        vemsign(
            wires:wires,
            timing_curve: timing_curve,
            rev_limit: 20000 * units.rpm
        )
    )
}

public node main {
    set_engine(v10())
}
