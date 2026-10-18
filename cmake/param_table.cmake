# The post-link parameter-table check (decision 0053, ground-segment unit 4b).
#
# marv_check_param_table(<target> <set>)
#   After every link of <target> (the m33 image, a SIL library, a planted control), tools/ci/check_param_table.py reads the
#   linked file and fails the build, removing the file, unless it holds exactly one parameter table and that table is the
#   one of <set> (marv_add_param_set): the record count, every record's name, type, unit and value bits, and the schema
#   hash, against <set>'s params_manifest.json. Every set defines the same marv::generated symbols, so a link that sees
#   two sets' archives takes whichever table the archive order offers first; this turns a wrong one into a build failure.
#   The command is the set's target property MARV_PARAM_TABLE_CHECK (fw/params/CMakeLists.txt), evaluated at generate
#   time, so a target defined before fw/params (the image, in fw/hal/target) can call this.
function(marv_check_param_table target set)
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND "$<TARGET_PROPERTY:${set}_generated,MARV_PARAM_TABLE_CHECK>" --elf "$<TARGET_FILE:${target}>"
            --delete-on-failure
    COMMENT "Checking the linked parameter table of ${target} against ${set}"
    COMMAND_EXPAND_LISTS
    VERBATIM)
endfunction()
