package org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm

enum class ArmStates {
    START,

    SAMPLE_PREINTAKE,               // when driving into submersible
    SAMPLE_INTAKE,                  // when intaking in sub
    SAMPLE_PREPOSITION_INTAKE,      // when intaking the prepositioned samples in auto
    SAMPLE_SCORE_BUCKET_HIGH,       // above the high bucket
    SAMPLE_SCORE_BUCKET_LOW,        // above the low bucket

    SPECIMEN_INTAKE_WALL,           // when intaking specimen from wall
    SPECIMEN_INTAKE_WALL_LIFTED,    // after intaking specimen from wall, left off wall
    SPECIMEN_ABOVE_HIGH_CHAMBER,    // above the high chamber
    SPECIMEN_ABOVE_LOW_CHAMBER,     // above the low chamber
    SPECIMEN_SCORE_HIGH_CHAMBER,    // pull down to score high chamber
    SPECIMEN_SCORE_LOW_CHAMBER,     // pul down to score low chamber

    PARK_OBSERVATION,               // arm state for parking in observation (maybe a reach)
    PARK_TOUCH_BAR                  // park touching the bar (L1 hang)
}