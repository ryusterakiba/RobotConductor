
(cl:in-package :asdf)

(defsystem "motion-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "music_commands" :depends-on ("_package_music_commands"))
    (:file "_package_music_commands" :depends-on ("_package"))
  ))