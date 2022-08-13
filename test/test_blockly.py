from pyri.sandbox import blockly_compiler
import io

def _do_blockly_compile_test(blockly_json, expected_pysrc):
    json_io = io.StringIO(blockly_json)
    output_io = io.StringIO()

    blockly_compiler.compile_blockly_file(json_io, output_io)
    output_io.seek(0)
    pysrc_str = output_io.read()
    print(pysrc_str)
    assert pysrc_str == expected_pysrc

def test_blockly_compiler_robotics():
    robotics_blockly_json = \
"""
{
  "blocks": {
    "languageVersion": 0,
    "blocks": [
      {
        "type": "procedures_defnoreturn",
        "id": "aaaaa",
        "x": 20,
        "y": 20,
        "icons": {
          "comment": {
            "text": "Describe this function...",
            "pinned": false,
            "height": 80,
            "width": 160
          }
        },
        "fields": {
          "NAME": "robot_mp_blocks_test"
        },
        "inputs": {
          "STACK": {
            "block": {
              "type": "robot_mp_set_active_robot",
              "id": "Y2?avR]J*|R*itXal}_c",
              "fields": {
                "ROBOT_NAME": "my_robot_mp"
              },
              "next": {
                "block": {
                  "type": "variables_set",
                  "id": "6]_WXrmG`1.~o2OFY,^]",
                  "fields": {
                    "VAR": {
                      "id": "7QKZav2?cnGV;~#4nfjE"
                    }
                  },
                  "inputs": {
                    "VALUE": {
                      "block": {
                        "type": "robot_mp_robot_pose",
                        "id": "n7~!EYnI%|xQXTKAzEBR",
                        "inputs": {
                          "TCP_POSE": {
                            "block": {
                              "type": "variables_get",
                              "id": "OkXbrh`%x~4782|lm$bM",
                              "fields": {
                                "VAR": {
                                  "id": "b:5[R0I3Y=/gdZ~i/O%@"
                                }
                              }
                            }
                          },
                          "JOINT_POSITION_SEED": {
                            "block": {
                              "type": "variables_get",
                              "id": "LJz8D.$U/Ru.s.zqgiP0",
                              "fields": {
                                "VAR": {
                                  "id": "E8T=s2@%-7)VU_$s;)?/"
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                  },
                  "next": {
                    "block": {
                      "type": "robot_mp_begin",
                      "id": "5t]+(4@!?MqeWu;ZqFU+",
                      "next": {
                        "block": {
                          "type": "robot_mp_move_absj",
                          "id": "-nQ(M/`=n!uW%dg,d6vN",
                          "fields": {
                            "SPEED": 1.23,
                            "BLEND_RADIUS": 0.334,
                            "FINE_POINT": true
                          },
                          "inputs": {
                            "JOINT_POSITION": {
                              "block": {
                                "type": "variables_get",
                                "id": "dJ(?c2PjoNutWD58/pQs",
                                "fields": {
                                  "VAR": {
                                    "id": "?@8UKgsnnfv/C#FV1;zY"
                                  }
                                }
                              }
                            }
                          },
                          "next": {
                            "block": {
                              "type": "robot_mp_movej",
                              "id": "^tpPvd%xK0?oIrhUqS$V",
                              "fields": {
                                "SPEED": 2.33,
                                "BLEND_RADIUS": 1.22,
                                "FINE_POINT": false
                              },
                              "inputs": {
                                "ROBOT_POSE": {
                                  "block": {
                                    "type": "variables_get",
                                    "id": "ErGEHg4%*=u*q83)?fDo",
                                    "fields": {
                                      "VAR": {
                                        "id": "7QKZav2?cnGV;~#4nfjE"
                                      }
                                    }
                                  }
                                }
                              },
                              "next": {
                                "block": {
                                  "type": "robot_mp_movel",
                                  "id": "uZKohdJH*g@D6?Ee0{MZ",
                                  "fields": {
                                    "SPEED": 1.11,
                                    "BLEND_RADIUS": 0.45,
                                    "FINE_POINT": true
                                  },
                                  "inputs": {
                                    "ROBOT_POSE": {
                                      "block": {
                                        "type": "variables_get",
                                        "id": "nh65?h#G6w,AcL[#l)u2",
                                        "fields": {
                                          "VAR": {
                                            "id": "b:5[R0I3Y=/gdZ~i/O%@"
                                          }
                                        }
                                      }
                                    }
                                  },
                                  "next": {
                                    "block": {
                                      "type": "robot_mp_movec",
                                      "id": "X%taP^%:~jzSYy8R~s]Y",
                                      "fields": {
                                        "SPEED": 0.9,
                                        "BLEND_RADIUS": 0.77,
                                        "FINE_POINT": true
                                      },
                                      "inputs": {
                                        "ROBOT_VIA_POSE": {
                                          "block": {
                                            "type": "variables_get",
                                            "id": "9#8t2!P.!HCF`^ZU{=-Y",
                                            "fields": {
                                              "VAR": {
                                                "id": "b:5[R0I3Y=/gdZ~i/O%@"
                                              }
                                            }
                                          }
                                        },
                                        "ROBOT_POSE": {
                                          "block": {
                                            "type": "variables_get",
                                            "id": "pSMj|]0SdTJ8:{Q9Wx$W",
                                            "fields": {
                                              "VAR": {
                                                "id": "E8T=s2@%-7)VU_$s;)?/"
                                              }
                                            }
                                          }
                                        }
                                      },
                                      "next": {
                                        "block": {
                                          "type": "robot_mp_execute",
                                          "id": "Pxra4)4~J#@Sd;GG!Rm2",
                                          "fields": {
                                            "WAIT": true
                                          },
                                          "next": {
                                            "block": {
                                              "type": "robot_mp_execute",
                                              "id": "Ow60K5v__01!RZdPch2f",
                                              "fields": {
                                                "WAIT": false
                                              }
                                            }
                                          }
                                        }
                                      }
                                    }
                                  }
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    ]
  },
  "variables": [
    {
      "name": "var4",
      "id": "7QKZav2?cnGV;~#4nfjE"
    },
    {
      "name": "var1",
      "id": "b:5[R0I3Y=/gdZ~i/O%@"
    },
    {
      "name": "var2",
      "id": "E8T=s2@%-7)VU_$s;)?/"
    },
    {
      "name": "var3",
      "id": "?@8UKgsnnfv/C#FV1;zY"
    }
  ]
}
"""
    expected_pysrc = \
        "# Describe this function...\n" \
        "def robot_mp_blocks_test():\n" \
        "\n" \
        "  robot_mp_set_active_robot(\"my_robot_mp\")\n" \
        "  var4 = robot_mp_robot_pose(var1, var2)\n" \
        "  robot_mp_begin()\n" \
        "  robot_mp_move_absj(var3, float(1.23), float(0.334), True)\n" \
        "  robot_mp_movej(var4, float(2.33), float(1.22), False)\n" \
        "  robot_mp_movel(var1, float(1.11), float(0.45), True)\n" \
        "  robot_mp_movec(var1, var2, float(0.9), float(0.77), True)\n" \
        "  robot_mp_execute(True)\n" \
        "  robot_mp_execute(False)\n"

    _do_blockly_compile_test(robotics_blockly_json, expected_pysrc)