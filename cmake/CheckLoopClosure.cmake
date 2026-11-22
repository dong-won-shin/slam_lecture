execute_process(COMMAND "${program}" "${data}/vocab_larger.yml.gz" "${data}"
  RESULT_VARIABLE status OUTPUT_VARIABLE output ERROR_VARIABLE errors)
if(NOT status STREQUAL "0")
  message(FATAL_ERROR "loop_closure exited with ${status}\n${errors}")
endif()

# Check the online decisions against the revisits in the bundled KITTI route.
set(expected none none none none none none none none 2.png 3.png 4.png none none 1.png)
set(query 0)
foreach(candidate IN LISTS expected)
  math(EXPR query "${query} + 1")
  set(line "Verified loop for ${query}.png: ${candidate}")
  string(FIND "${output}" "${line}\n" position)
  if(position EQUAL -1)
    message(FATAL_ERROR "Expected '${line}' in loop_closure output\n${output}")
  endif()
endforeach()
