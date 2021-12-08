;; Auto-generated. Do not edit!


(when (boundp 'motion::music_commands)
  (if (not (find-package "MOTION"))
    (make-package "MOTION"))
  (shadow 'music_commands (find-package "MOTION")))
(unless (find-package "MOTION::MUSIC_COMMANDS")
  (make-package "MOTION::MUSIC_COMMANDS"))

(in-package "ROS")
;;//! \htmlinclude music_commands.msg.html


(defclass motion::music_commands
  :super ros::object
  :slots (_tempo _right_arm_motions _left_arm_motions ))

(defmethod motion::music_commands
  (:init
   (&key
    ((:tempo __tempo) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:right_arm_motions __right_arm_motions) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:left_arm_motions __left_arm_motions) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _tempo __tempo)
   (setq _right_arm_motions __right_arm_motions)
   (setq _left_arm_motions __left_arm_motions)
   self)
  (:tempo
   (&optional __tempo)
   (if __tempo (setq _tempo __tempo)) _tempo)
  (:right_arm_motions
   (&optional __right_arm_motions)
   (if __right_arm_motions (setq _right_arm_motions __right_arm_motions)) _right_arm_motions)
  (:left_arm_motions
   (&optional __left_arm_motions)
   (if __left_arm_motions (setq _left_arm_motions __left_arm_motions)) _left_arm_motions)
  (:serialization-length
   ()
   (+
    ;; float64[] _tempo
    (* 8    (length _tempo)) 4
    ;; string[] _right_arm_motions
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _right_arm_motions)) 4
    ;; string[] _left_arm_motions
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _left_arm_motions)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _tempo
     (write-long (length _tempo) s)
     (dotimes (i (length _tempo))
       (sys::poke (elt _tempo i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string[] _right_arm_motions
     (write-long (length _right_arm_motions) s)
     (dolist (elem _right_arm_motions)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _left_arm_motions
     (write-long (length _left_arm_motions) s)
     (dolist (elem _left_arm_motions)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _tempo
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tempo (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tempo i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string[] _right_arm_motions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _right_arm_motions (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _right_arm_motions i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _left_arm_motions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _left_arm_motions (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _left_arm_motions i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get motion::music_commands :md5sum-) "ed9308da9714234f1e05e7985bf4f8c3")
(setf (get motion::music_commands :datatype-) "motion/music_commands")
(setf (get motion::music_commands :definition-)
      "float64[] tempo
string[] right_arm_motions
string[] left_arm_motions
")



(provide :motion/music_commands "ed9308da9714234f1e05e7985bf4f8c3")


